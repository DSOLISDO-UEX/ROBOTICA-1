#include "controller.h"
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>

Controller::Controller()
    : params_(Params{})
{
}

Controller::Controller(const Params& params)
    : params_(params)
{
}

void Controller::setParams(const Params& params)
{
    params_ = params;
}

const Controller::Params& Controller::getParams() const
{
    return params_;
}

std::tuple<double, double, double> Controller::forwardModel(double adv, double rot, double dt)
{
    if (std::abs(rot) < 1e-4)
    {
        return {0.0, adv * dt, 0.0};
    }

    const double theta = rot * dt;
    // En coordenadas del robot: +Y es avance longitudinal, +X lateral derecha, -X lateral izquierda
    // rot > 0 gira antihorario (hacia -X), rot < 0 gira horario (hacia +X)
    const double x = (adv / rot) * (std::cos(theta) - 1.0);
    const double y = (adv / rot) * std::sin(theta);
    return {x, y, theta};
}

std::vector<QPointF> Controller::generateArc(double adv, double rot, double duration, int steps) const
{
    std::vector<QPointF> arc;
    arc.reserve(steps + 1);
    arc.emplace_back(0.0, 0.0);

    for (int i = 1; i <= steps; ++i)
    {
        const double t = duration * (static_cast<double>(i) / steps);
        const auto [x, y, _] = forwardModel(adv, rot, t);
        arc.emplace_back(static_cast<float>(x), static_cast<float>(y));
    }
    return arc;
}

std::vector<QPointF> Controller::extractObstaclePoints(const RoboCompLidar3D::TPoints& points,
                                                      const QPointF& personPos) const
{
    std::vector<QPointF> obstacles;
    obstacles.reserve(std::min<size_t>(points.size(), 600));

    const float personExclusionSq = params_.personBodyRadiusMm * params_.personBodyRadiusMm;

    for (const auto& pt : points)
    {
        // Descartar retornos sobre el chasis propio del robot (< 220 mm)
        if (pt.distance2d < 220.0f) continue;
        // Descartar obstáculos demasiado lejanos para la planificación reactiva local (> 4000 mm)
        if (pt.distance2d > 4000.0f) continue;
        // Enfocar en el semicírculo frontal y lateral relevante (-400 mm < y < 3500 mm, |x| < 2500 mm)
        if (pt.y < -400.0f || pt.y > 3500.0f || std::abs(pt.x) > 2500.0f) continue;

        // Comprobar si el punto pertenece al cuerpo de la persona objetivo
        const float dxPerson = pt.x - personPos.x();
        const float dyPerson = pt.y - personPos.y();
        if ((dxPerson * dxPerson + dyPerson * dyPerson) < personExclusionSq)
        {
            continue; // Excluir puntos de la persona para no repelerse de ella
        }

        obstacles.emplace_back(pt.x, pt.y);
    }
    return obstacles;
}

std::pair<bool, float> Controller::evaluateObstacleClearance(const std::vector<QPointF>& arc,
                                                            const std::vector<QPointF>& obstacles,
                                                            float collisionThreshold) const
{
    if (obstacles.empty())
    {
        return {false, 99999.0f};
    }

    float minDistance = 99999.0f;
    const float colThreshSq = collisionThreshold * collisionThreshold;

    for (const auto& arcPt : arc)
    {
        for (const auto& obsPt : obstacles)
        {
            const float dx = arcPt.x() - obsPt.x();
            const float dy = arcPt.y() - obsPt.y();
            const float distSq = dx * dx + dy * dy;

            if (distSq < colThreshSq)
            {
                return {true, std::sqrt(distSq)}; // Colisión detectada a lo largo del arco
            }

            if (distSq < minDistance * minDistance)
            {
                minDistance = std::sqrt(distSq);
            }
        }
    }

    return {false, minDistance};
}

Controller::Output Controller::update(double personAngle,
                                      double personDist,
                                      double radialVel,
                                      double personVelX,
                                      double personVelY,
                                      const RoboCompLidar3D::TPoints& lidarPoints)
{
    Output out;

    // 1. Posición cartesiana actual y predicha de la persona en el marco del robot
    const float currentPersonX = static_cast<float>(personDist * std::sin(personAngle));
    const float currentPersonY = static_cast<float>(personDist * std::cos(personAngle));
    const QPointF currentPersonPos(currentPersonX, currentPersonY);

    // Predicción de la persona al final del horizonte de simulación
    const float targetPersonX = currentPersonX + static_cast<float>(personVelX * params_.simTimeSec);
    const float targetPersonY = currentPersonY + static_cast<float>(personVelY * params_.simTimeSec);

    // 2. Extraer puntos de obstáculos estáticos (cajas, paredes), ignorando a la persona
    const auto obstacles = extractObstaclePoints(lidarPoints, currentPersonPos);

    // 3. Comando de referencia del controlador proporcional (línea base de seguimiento)
    const float distError = static_cast<float>(personDist) - params_.targetDistMm;
    const double absAngle = std::abs(personAngle);

    float alignmentFactor = 0.0f;
    if (absAngle < 0.436) // < 25 grados
    {
        alignmentFactor = 1.0f;
    }
    else if (absAngle < 1.40) // < 80 grados
    {
        alignmentFactor = std::max(0.25f, static_cast<float>(std::cos(absAngle)));
    }
    else
    {
        alignmentFactor = 0.0f;
    }

    float propAdv = (distError * params_.kpDist + static_cast<float>(radialVel) * params_.kdVel) * alignmentFactor;
    propAdv = std::clamp(propAdv, params_.minAdvSpeed, params_.maxAdvSpeed);

    float propRot = std::clamp(static_cast<float>(-personAngle * params_.kpRot),
                               -params_.maxRotSpeed, params_.maxRotSpeed);

    // 4. Muestreo del espacio de velocidades DWA: adv{0 .. 1000} x rot{-1 .. 1}
    const float collisionThreshold = params_.robotRadiusMm + params_.safetyMarginMm;

    struct Candidate
    {
        float adv = 0.0f;
        float rot = 0.0f;
        float cost = std::numeric_limits<float>::max();
        float minObsDist = 99999.0f;
        std::vector<QPointF> arc;
    };

    std::vector<Candidate> validCandidates;
    validCandidates.reserve(params_.advSamples * params_.rotSamples + 1);

    const float advStep = (params_.advSamples > 1)
                              ? (params_.maxAdvSpeed - params_.minAdvSpeed) / (params_.advSamples - 1)
                              : 0.0f;
    const float rotStep = (params_.rotSamples > 1)
                              ? (2.0f * params_.maxRotSpeed) / (params_.rotSamples - 1)
                              : 0.0f;

    // Generar candidatos de la rejilla
    for (int i = 0; i < params_.advSamples; ++i)
    {
        const float candAdv = params_.minAdvSpeed + i * advStep;
        for (int j = 0; j < params_.rotSamples; ++j)
        {
            const float candRot = -params_.maxRotSpeed + j * rotStep;

            const auto arc = generateArc(candAdv, candRot, params_.simTimeSec, params_.simSteps);
            const auto [collision, minObsDist] = evaluateObstacleClearance(arc, obstacles, collisionThreshold);

            if (!collision)
            {
                validCandidates.push_back({candAdv, candRot, 0.0f, minObsDist, arc});
            }
        }
    }

    // Incluir explícitamente el comando proporcional como candidato si no colisiona
    {
        const auto propArc = generateArc(propAdv, propRot, params_.simTimeSec, params_.simSteps);
        const auto [propCollision, propMinDist] = evaluateObstacleClearance(propArc, obstacles, collisionThreshold);
        if (!propCollision)
        {
            validCandidates.push_back({propAdv, propRot, 0.0f, propMinDist, propArc});
        }
    }

    // 5. Evaluación de la función de coste para cada candidato no colisionante
    float bestCost = std::numeric_limits<float>::max();
    Candidate bestCandidate;
    bool foundCandidate = false;

    for (auto& cand : validCandidates)
    {
        const auto [endX, endY, endTheta] = forwardModel(cand.adv, cand.rot, params_.simTimeSec);

        // Term 1: Distancia final del arco a la persona objetivo
        const float dxToEnd = targetPersonX - static_cast<float>(endX);
        const float dyToEnd = targetPersonY - static_cast<float>(endY);
        const float distToEnd = std::sqrt(dxToEnd * dxToEnd + dyToEnd * dyToEnd);

        // Term 2: El final del arco está a 1m de la persona (|dist - 1000|)
        const float costTargetDist = std::abs(distToEnd - params_.targetDistMm) / params_.targetDistMm;

        // Term 3: El final del arco encara a la persona
        // Dirección hacia la persona desde el extremo del arco (en coordenadas robot: ángulo desde +Y hacia -X)
        const double bearingToPerson = std::atan2(-dxToEnd, dyToEnd);
        const double headingDiff = std::abs(std::remainder(endTheta - bearingToPerson, 2.0 * M_PI));
        const float costHeading = static_cast<float>(headingDiff / M_PI);

        // Term 4: Distancia a los obstáculos (maximizar holgura -> penalizar cercanía)
        float costObstacle = 0.0f;
        if (cand.minObsDist < params_.obstacleInfluenceDistMm)
        {
            costObstacle = (params_.obstacleInfluenceDistMm - cand.minObsDist) /
                           (params_.obstacleInfluenceDistMm - collisionThreshold);
            costObstacle = std::clamp(costObstacle, 0.0f, 2.0f);
        }

        // Term 5: Preferir movimientos rápidos hacia el objetivo
        const float costSpeed = 1.0f - (cand.adv / params_.maxAdvSpeed);

        // Term 6: Progreso general hacia la persona
        const float costDistToPerson = distToEnd / 4000.0f;

        // Term 7: Alineación suave con el controlador proporcional
        const float costProp = 0.5f * (std::abs(cand.adv - propAdv) / params_.maxAdvSpeed) +
                               0.5f * (std::abs(cand.rot - propRot) / (2.0f * params_.maxRotSpeed));

        // Coste total ponderado
        cand.cost = params_.weightTargetDist * costTargetDist +
                    params_.weightHeading * costHeading +
                    params_.weightObstacle * costObstacle +
                    params_.weightSpeed * costSpeed +
                    params_.weightDistToPerson * costDistToPerson +
                    params_.weightPropAlignment * costProp;

        if (cand.cost < bestCost)
        {
            bestCost = cand.cost;
            bestCandidate = cand;
            foundCandidate = true;
        }
    }

    // 6. Asignación del comando resultante o fallback jerárquico de seguridad
    if (foundCandidate)
    {
        out.advSpeed = bestCandidate.adv;
        out.rotSpeed = bestCandidate.rot;
        out.minObsDist = bestCandidate.minObsDist;
        out.bestArc = bestCandidate.arc;

        // Indicar si DWA está maniobrando para evitar obstáculos
        const bool diffFromProp = (std::abs(out.advSpeed - propAdv) > 100.0f ||
                                   std::abs(out.rotSpeed - propRot) > 0.2f);
        out.isAvoiding = (out.minObsDist < params_.obstacleInfluenceDistMm) && diffFromProp;
    }
    else
    {
        // En caso de que todos los arcos con avance estén bloqueados:
        // Prioridad jerárquica: EVITAR COLISIÓN (velocidad lineal cero, rotar sobre sí mismo hacia la zona más despejada)
        out.advSpeed = 0.0f;

        // Evaluar giros puros in-situ para despejar el camino
        float bestClearance = -1.0f;
        float bestPivotRot = 0.0f;
        for (float testRot : {-1.0f, -0.6f, -0.3f, 0.3f, 0.6f, 1.0f})
        {
            const auto pivotArc = generateArc(0.0, testRot, params_.simTimeSec, 3);
            const auto [_, clearance] = evaluateObstacleClearance(pivotArc, obstacles, collisionThreshold);
            if (clearance > bestClearance)
            {
                bestClearance = clearance;
                bestPivotRot = testRot;
            }
        }

        out.rotSpeed = bestPivotRot;
        out.isAvoiding = true;
        out.minObsDist = bestClearance;
        out.bestArc = generateArc(0.0, out.rotSpeed, params_.simTimeSec, params_.simSteps);
    }

    // Muestreo de arcos para visualización (tomar un subconjunto representativo)
    for (size_t k = 0; k < validCandidates.size(); k += std::max<size_t>(1, validCandidates.size() / 15))
    {
        out.candidateArcs.push_back(validCandidates[k].arc);
    }

    return out;
}
