#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

// #define HIBERNATION_ENABLED

#ifdef emit
#undef emit
#endif

#include <algorithm>
#include <optional>
#include <tuple>
#include <vector>
#include <genericworker.h>
#include <abstract_graphic_viewer/abstract_graphic_viewer.h>
#include <Eigen/Dense>
#include "personTracker.h"

/**
 * @brief Componente principal para la detección, estimación de estado y seguimiento de una persona.
 */
class SpecificWorker : public GenericWorker
{
    Q_OBJECT

public:
    /**
     * @brief Constructor de SpecificWorker.
     * @param configLoader Cargador de configuración del componente.
     * @param tprx Tupla de proxies de comunicaciones de RoboComp.
     * @param startupCheck Indica si se ejecuta en modo de comprobación inicial.
     */
    SpecificWorker(const ConfigLoader& configLoader, TuplePrx tprx, bool startupCheck);

    /**
     * @brief Destructor de SpecificWorker.
     */
    ~SpecificWorker() override;

public slots:
    /**
     * @brief Inicializa el visor gráfico y la geometría del robot.
     */
    void initialize() override;

    /**
     * @brief Ciclo principal: compensa egomoción, fusiona LiDAR/cámara, actualiza Kalman y comanda velocidades.
     */
    void compute() override;

    /**
     * @brief Gestiona el estado de emergencia del componente.
     */
    void emergency() override;

    /**
     * @brief Restaura el componente tras un estado de emergencia.
     */
    void restore() override;

    /**
     * @brief Realiza la comprobación de arranque del componente.
     * @return Código de retorno del chequeo.
     */
    int startup_check();

private:
    bool startupCheckFlag_ = false;
    QRectF dimensions_;
    AbstractGraphicViewer* viewer_ = nullptr;
    QGraphicsItem* robotPolygon_ = nullptr;
    PersonTracker tracker_;
    double timeLastCycle_ = -1.0;
    float lastAdvCmd_ = 0.0f;
    float lastRotCmd_ = 0.0f;

    /**
     * @brief Detiene la base omnidireccional y pone a cero los registros de comandos previos.
     */
    void StopRobot();

    /**
     * @brief Filtra los puntos 3D del LiDAR dentro de un sector angular y por encima de una distancia mínima.
     * @param points Nube de puntos de entrada.
     * @param minAngle Ángulo mínimo del cono (rad).
     * @param maxAngle Ángulo máximo del cono (rad).
     * @param minDistance Distancia mínima 2D permitida (mm).
     * @return Puntos filtrados dentro del cono.
     */
    RoboCompLidar3D::TPoints FilterCone(const RoboCompLidar3D::TPoints& points, float minAngle,
                                        float maxAngle, float minDistance = 0.0f);

    /**
     * @brief Proyecta puntos LiDAR sobre la imagen equirectangular 360 y calcula la profundidad mínima en la ROI.
     * @param points Nube de puntos del LiDAR.
     * @param camTransform Transformación rígida del marco del LiDAR al marco de la cámara.
     * @param personAngle Ángulo estimado hacia la persona en la imagen (rad).
     * @param image Imagen 360 RGB actual.
     * @param roi Bounding box 2D de la persona detectada.
     * @param azimuthSign Signo de orientación acimutal.
     * @param azimuthOffset Desfase angular acimutal (rad).
     * @return Tupla opcional con los puntos proyectados en la ROI y la profundidad mínima (mm).
     */
    std::optional<std::tuple<std::vector<QPointF>, float>> ProjectLidarToCamera(
        const RoboCompLidar3D::TPoints& points,
        const Eigen::Isometry3d& camTransform,
        double personAngle,
        const RoboCompCamera360RGB::TImage& image,
        const QRect& roi,
        double azimuthSign,
        double azimuthOffset);

    /**
     * @brief Renderiza la imagen 360 con la ROI, las guías de centrado y las proyecciones del LiDAR.
     * @param raw Imagen RGB de entrada.
     * @param roi Rectángulo delimitador de la persona.
     * @param projectedPoints Puntos 2D proyectados sobre la imagen.
     * @param targetLabel Widget QLabel donde mostrar el resultado.
     */
    void DrawImage(const RoboCompCamera360RGB::TImage& raw, const QRect& roi,
                   const std::vector<QPointF>& projectedPoints, QLabel* targetLabel);

    /**
     * @brief Dibuja la posición y datos estimados de la persona en la escena 2D del visor.
     * @param personAngle Ángulo estimado de la persona (rad).
     * @param distance Distancia estimada de la persona (mm).
     * @param scene Escena gráfica de Qt.
     */
    void DrawPersonInLidar(float personAngle, float distance, QGraphicsScene* scene);

    /**
     * @brief Dibuja los puntos de la nube LiDAR en la escena 2D del visor.
     * @param points Nube de puntos LiDAR.
     * @param scene Escena gráfica de Qt.
     */
    void DrawLidar(const RoboCompLidar3D::TPoints& points, QGraphicsScene* scene);
};

#endif // SPECIFICWORKER_H