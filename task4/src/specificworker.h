#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

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
#include "controller.h"

/**
 * @brief Componente principal para el seguimiento de personas y evasión de obstáculos con DWA.
 */
class SpecificWorker : public GenericWorker
{
    Q_OBJECT

public:
    /**
     * @brief Constructor de SpecificWorker.
     * @param config_loader Cargador de configuración del componente.
     * @param tprx Tupla de proxies de comunicaciones de RoboComp.
     * @param startup_check Indica si se ejecuta en modo de comprobación inicial.
     */
    SpecificWorker(const ConfigLoader& config_loader, TuplePrx tprx, bool startup_check);

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
     * @brief Ciclo principal: compensa egomoción, fusiona LiDAR/cámara, actualiza Kalman y ejecuta controlador DWA.
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
    bool startup_check_flag_ = false;
    AbstractGraphicViewer* viewer_ = nullptr;
    PersonTracker tracker_;
    Controller controller_;
    double time_last_cycle_ = -1.0;
    float last_adv_cmd_ = 0.0f;
    float last_rot_cmd_ = 0.0f;

    /**
     * @brief Detiene la base omnidireccional y pone a cero los registros de comandos previos.
     */
    void StopRobot();

    /**
     * @brief Filtra los puntos 3D del LiDAR dentro de un sector angular y por encima de una distancia mínima.
     * @param points Nube de puntos de entrada.
     * @param min_angle Ángulo mínimo del cono (rad).
     * @param max_angle Ángulo máximo del cono (rad).
     * @param min_distance Distancia mínima 2D permitida (mm).
     * @return Puntos filtrados dentro del cono.
     */
    RoboCompLidar3D::TPoints FilterCone(const RoboCompLidar3D::TPoints& points, float min_angle,
                                        float max_angle, float min_distance = 0.0f);

    /**
     * @brief Proyecta puntos LiDAR sobre la imagen equirectangular 360 y calcula la profundidad mínima en la ROI.
     */
    std::optional<std::tuple<std::vector<QPointF>, float>> ProjectLidarToCamera(
        const RoboCompLidar3D::TPoints& points,
        const Eigen::Isometry3d& cam_transform,
        double person_angle,
        const RoboCompCamera360RGB::TImage& image,
        const QRect& roi,
        double azimuth_sign,
        double azimuth_offset);

    /**
     * @brief Renderiza la imagen 360 con la ROI, las guías de centrado y las proyecciones del LiDAR.
     */
    void DrawImage(const RoboCompCamera360RGB::TImage& raw, const QRect& roi,
                   const std::vector<QPointF>& projected_points, QLabel* target_label);

    /**
     * @brief Dibuja la posición y datos estimados de la persona en la escena 2D del visor.
     */
    void DrawPersonInLidar(float person_angle, float distance, QGraphicsScene* scene);

    /**
     * @brief Dibuja los puntos de la nube LiDAR en la escena 2D del visor.
     */
    void DrawLidar(const RoboCompLidar3D::TPoints& points, QGraphicsScene* scene);

    /**
     * @brief Dibuja los arcos de trayectoria evaluados y seleccionados por DWA en la escena 2D.
     */
    void DrawDwaArcs(const Controller::Output& output, QGraphicsScene* scene);
};

#endif // SPECIFICWORKER_H
