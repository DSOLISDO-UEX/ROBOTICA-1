#ifndef PERSON_TRACKER_H
#define PERSON_TRACKER_H

#include <Eigen/Dense>
#include <tuple>

/**
 * @brief Filtro de Kalman lineal con compensación de egomoción para seguimiento 2D de una persona.
 */
class PersonTracker
{
public:
    PersonTracker() = default;

    /**
     * @brief Reinicia el estado del filtro, la matriz de covarianza y los contadores.
     */
    void Reset();

    /**
     * @brief Compensa el desplazamiento y rotación propios del robot en el estado estimado.
     * @param adv_speed Velocidad lineal comandada (mm/s).
     * @param rot_speed Velocidad angular comandada (rad/s).
     * @param dt_ego Intervalo de tiempo transcurrido (s).
     */
    void EgoCompensate(double adv_speed, double rot_speed, double dt_ego);

    /**
     * @brief Ejecuta la etapa de predicción de velocidad constante del filtro de Kalman.
     * @param dt Diferencial de tiempo (s).
     */
    void Predict(double dt);

    /**
     * @brief Actualiza el filtro con una nueva observación polar tras validar con puerta de Mahalanobis.
     * @param angle Ángulo medido hacia la persona (rad).
     * @param distance Distancia medida hacia la persona (mm).
     * @param time_now Marca de tiempo actual (s).
     */
    void ProcessMeasurement(double angle, double distance, double time_now);

    /**
     * @brief Predice el estado en ausencia de medidas válidas hasta el límite temporal de caída.
     * @param dt_ego Diferencial de tiempo del ciclo (s).
     * @param time_now Marca de tiempo actual (s).
     * @return true si la predicción es válida, false si expiró el tiempo y se reinició el filtro.
     */
    bool StepWithoutMeasurement(double dt_ego, double time_now);

    /**
     * @brief Obtiene la posición estimada actual en coordenadas polares.
     * @return Tupla con ángulo (rad) y distancia (mm).
     */
    std::tuple<double, double> GetPolarEstimate() const;

    /**
     * @brief Obtiene la posición cartesiana estimada en el marco del robot (mm).
     */
    Eigen::Vector2d GetCartesianPosition() const { return state_.head<2>(); }

    /**
     * @brief Obtiene la velocidad cartesiana estimada en el marco del robot (mm/s).
     */
    Eigen::Vector2d GetCartesianVelocity() const { return state_.tail<2>(); }

    /**
     * @brief Calcula la velocidad radial proyectada sobre la línea de visión hacia el objetivo.
     * @return Velocidad radial (mm/s).
     */
    double GetRadialVelocity() const;

    /**
     * @brief Indica si el filtro se encuentra inicializado con seguimiento activo.
     * @return true si está inicializado.
     */
    bool IsInitialized() const;

private:
    Eigen::Vector4d state_ = Eigen::Vector4d::Zero();
    Eigen::Matrix4d covariance_ = Eigen::Matrix4d::Zero();
    bool is_initialized_ = false;
    double last_meas_time_ = -1.0;
    int consecutive_rejections_ = 0;

    /**
     * @brief Proyecta la matriz de covarianza de ruido de observación polar al espacio cartesiano.
     * @param angle Ángulo medido (rad).
     * @param distance Distancia medida (mm).
     * @return Matriz de covarianza R 2x2.
     */
    static Eigen::Matrix2d ComputeR(double angle, double distance);

    /**
     * @brief Inicializa el vector de estado y la matriz de covarianza a partir de una medida polar.
     * @param angle Ángulo inicial (rad).
     * @param distance Distancia inicial (mm).
     * @param time_now Marca de tiempo actual (s).
     */
    void InitFromMeasurement(double angle, double distance, double time_now);

    /**
     * @brief Fuerza la simetría numérica de la matriz de covarianza P.
     */
    void SymmetrizeP();
};

#endif // PERSON_TRACKER_H
