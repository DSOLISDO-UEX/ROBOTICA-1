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
        // Descartar retornos sobre el chasis y ruedas propias del robot
        if (pt.distance2d < params_.minSelfChassisDistMm) continue;
        if (std::abs(pt.x) < 320.0f && std::abs(pt.y) < 300.0f) continue;
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
        // Para giros in-situ (adv == 0), el umbral es el radio físico del robot (285 mm).
        // Para trayectorias con avance lineal (adv > 0), se incluye el margen de seguridad (320 mm).
        const float currentThresh = (candAdv < 1.0f) ? params_.robotRadiusMm : collisionThreshold;
        for (int j = 0; j < params_.rotSamples; ++j)
        {
            const float candRot = -params_.maxRotSpeed + j * rotStep;

            const auto arc = generateArc(candAdv, candRot, params_.simTimeSec, params_.simSteps);
            const auto [collision, minObsDist] = evaluateObstacleClearance(arc, obstacles, currentThresh);

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
    // Detectar si el corredor frontal inmediato está bloqueado por un obstáculo
    // (Solo activo cuando el objetivo está en el cono frontal y hay un obstáculo delante)
    bool forwardBlocked = false;
    float obsCentroidX = 0.0f;
    int blockedCount = 0;
    if (alignmentFactor > 0.0f)
    {
        for (const auto& obs : obstacles)
        {
            if (obs.y() > 0.0f && obs.y() < 1600.0f && std::abs(obs.x()) < 420.0f)
            {
                forwardBlocked = true;
                obsCentroidX += obs.x();
                blockedCount++;
            }
        }
        if (blockedCount > 0)
        {
            obsCentroidX /= static_cast<float>(blockedCount);
        }
    }

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
        const float weightHeadingEff = forwardBlocked ? 0.2f : params_.weightHeading;

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

        // Term 7: Alineación suave con el controlador proporcional (desactivado durante evasión frontal)
        const float costProp = forwardBlocked
                                   ? 0.0f
                                   : (0.5f * (std::abs(cand.adv - propAdv) / params_.maxAdvSpeed) +
                                      0.5f * (std::abs(cand.rot - propRot) / (2.0f * params_.maxRotSpeed)));

        // Term 8: Histéresis de giro para prevenir oscilaciones rápidas izquierda/derecha
        float costHysteresis = 0.0f;
        if (std::abs(cand.rot) > 0.1f && std::abs(lastChosenRot_) > 0.1f)
        {
            if (cand.rot * lastChosenRot_ < 0.0f)
            {
                costHysteresis = 0.6f; // Penalizar cambios bruscos de sentido de giro
            }
        }

        // Term 9: Prioridad de evasión jerárquica cuando el frente está bloqueado
        float costCorridor = 0.0f;
        if (forwardBlocked)
        {
            if (cand.adv < 50.0f || std::abs(cand.rot) < 0.6f)
            {
                costCorridor = 4.0f; // Penalizar detenerse o avanzar recto hacia el obstáculo
            }
            else if (obsCentroidX <= 0.0f && cand.rot < -0.5f)
            {
                costCorridor = -2.0f - 0.5f * std::abs(cand.rot); // Obstáculo a la izquierda -> esquivar por la derecha
            }
            else if (obsCentroidX > 0.0f && cand.rot > 0.5f)
            {
                costCorridor = -2.0f - 0.5f * std::abs(cand.rot); // Obstáculo a la derecha -> esquivar por la izquierda
            }
        }

        // Coste total ponderado
        cand.cost = params_.weightTargetDist * costTargetDist +
                    weightHeadingEff * costHeading +
                    params_.weightObstacle * costObstacle +
                    params_.weightSpeed * costSpeed +
                    params_.weightDistToPerson * costDistToPerson +
                    params_.weightPropAlignment * costProp +
                    costHysteresis +
                    costCorridor;

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
        if (bestCandidate.adv >= 100.0f)
        {
            lastPivotRot_ = 0.0f; // Reiniciar histéresis una vez que el robot avanza fluidamente
        }
        lastChosenRot_ = bestCandidate.rot;
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
        // En caso de que todos los candidatos estén restringidos por cercanía:
        // Prioridad de seguridad: velocidad lineal cero y rotar in-situ para encarar a la persona
        out.advSpeed = 0.0f;
        const double bearingToGoal = std::atan2(-targetPersonX, targetPersonY);
        out.rotSpeed = (bearingToGoal >= 0.0) ? params_.maxRotSpeed : -params_.maxRotSpeed;
        out.isAvoiding = true;
        out.minObsDist = params_.robotRadiusMm;
        out.bestArc = generateArc(0.0, out.rotSpeed, params_.simTimeSec, params_.simSteps);
    }

    // Muestreo de arcos para visualización (tomar un subconjunto representativo)
    for (size_t k = 0; k < validCandidates.size(); k += std::max<size_t>(1, validCandidates.size() / 15))
    {
        out.candidateArcs.push_back(validCandidates[k].arc);
    }

    return out;
}
