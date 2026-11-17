#include "controller.h"

#include <algorithm>
#include <cmath>
#include <limits>

Controller::Controller() : params_(Params{}) {}

Controller::Controller(const Params& params) : params_(params) {}

void Controller::SetParams(const Params& params)
{
    params_ = params;
}

const Controller::Params& Controller::GetParams() const
{
    return params_;
}

std::tuple<double, double, double> Controller::ForwardModel(double adv, double rot, double dt)
{
    if (std::abs(rot) < 1e-4)
    {
        return {0.0, adv * dt, 0.0};
    }

    const double theta = rot * dt;
    // Robot frame: +Y is longitudinal forward, +X is lateral right, -X is lateral left.
    // rot > 0 turns counter-clockwise (towards -X), rot < 0 turns clockwise (towards +X).
    const double x = (adv / rot) * (std::cos(theta) - 1.0);
    const double y = (adv / rot) * std::sin(theta);
    return {x, y, theta};
}

std::vector<QPointF> Controller::GenerateArc(double adv, double rot, double duration, int steps) const
{
    std::vector<QPointF> arc;
    arc.reserve(steps + 1);
    arc.emplace_back(0.0, 0.0);

    for (int i = 1; i <= steps; ++i)
    {
        const double t = duration * (static_cast<double>(i) / steps);
        const auto [x, y, _] = ForwardModel(adv, rot, t);
        arc.emplace_back(static_cast<float>(x), static_cast<float>(y));
    }
    return arc;
}

std::vector<QPointF> Controller::ExtractObstaclePoints(const RoboCompLidar3D::TPoints& points,
                                                      const QPointF& person_pos) const
{
    std::vector<QPointF> obstacles;
    obstacles.reserve(std::min<size_t>(points.size(), 600));

    const float person_exclusion_sq = params_.person_body_radius_mm * params_.person_body_radius_mm;

    for (const auto& pt : points)
    {
        // Filter out robot chassis self-returns and wheelpods
        if (pt.distance2d < params_.min_self_chassis_dist_mm) continue;
        if (std::abs(pt.x) < 320.0f && std::abs(pt.y) < 300.0f) continue;
        // Filter out obstacles beyond local planning horizon
        if (pt.distance2d > 4000.0f) continue;
        // Relevant frontal and lateral semi-plane (-400 mm < y < 3500 mm, |x| < 2500 mm)
        if (pt.y < -400.0f || pt.y > 3500.0f || std::abs(pt.x) > 2500.0f) continue;

        // Exclude target person points
        const float dx_person = pt.x - person_pos.x();
        const float dy_person = pt.y - person_pos.y();
        if ((dx_person * dx_person + dy_person * dy_person) < person_exclusion_sq)
        {
            continue;
        }

        obstacles.emplace_back(pt.x, pt.y);
    }
    return obstacles;
}

std::pair<bool, float> Controller::EvaluateObstacleClearance(const std::vector<QPointF>& arc,
                                                            const std::vector<QPointF>& obstacles,
                                                            float collision_threshold) const
{
    if (obstacles.empty())
    {
        return {false, 99999.0f};
    }

    float min_distance = 99999.0f;
    const float col_thresh_sq = collision_threshold * collision_threshold;

    for (const auto& arc_pt : arc)
    {
        for (const auto& obs_pt : obstacles)
        {
            const float dx = arc_pt.x() - obs_pt.x();
            const float dy = arc_pt.y() - obs_pt.y();
            const float dist_sq = dx * dx + dy * dy;

            if (dist_sq < col_thresh_sq)
            {
                return {true, std::sqrt(dist_sq)};
            }

            if (dist_sq < min_distance * min_distance)
            {
                min_distance = std::sqrt(dist_sq);
            }
        }
    }

    return {false, min_distance};
}

Controller::Output Controller::Update(double person_angle,
                                      double person_dist,
                                      double radial_vel,
                                      double person_vel_x,
                                      double person_vel_y,
                                      const RoboCompLidar3D::TPoints& lidar_points)
{
    Output out;

    const float current_person_x = static_cast<float>(person_dist * std::sin(person_angle));
    const float current_person_y = static_cast<float>(person_dist * std::cos(person_angle));
    const QPointF current_person_pos(current_person_x, current_person_y);

    const float target_person_x = current_person_x + static_cast<float>(person_vel_x * params_.sim_time_sec);
    const float target_person_y = current_person_y + static_cast<float>(person_vel_y * params_.sim_time_sec);

    const auto obstacles = ExtractObstaclePoints(lidar_points, current_person_pos);

    // Proportional baseline controller
    const float dist_error = static_cast<float>(person_dist) - params_.target_dist_mm;
    const double abs_angle = std::abs(person_angle);

    float alignment_factor = 0.0f;
    if (abs_angle < 0.436) // < 25 degrees
    {
        alignment_factor = 1.0f;
    }
    else if (abs_angle < 1.40) // < 80 degrees
    {
        alignment_factor = std::max(0.25f, static_cast<float>(std::cos(abs_angle)));
    }

    float prop_adv = (dist_error * params_.kp_dist + static_cast<float>(radial_vel) * params_.kd_vel) * alignment_factor;
    prop_adv = std::clamp(prop_adv, params_.min_adv_speed, params_.max_adv_speed);

    const float prop_rot = std::clamp(static_cast<float>(-person_angle * params_.kp_rot),
                                      -params_.max_rot_speed, params_.max_rot_speed);

    const float collision_threshold = params_.robot_radius_mm + params_.safety_margin_mm;

    struct Candidate
    {
        float adv = 0.0f;
        float rot = 0.0f;
        float cost = std::numeric_limits<float>::max();
        float min_obs_dist = 99999.0f;
        std::vector<QPointF> arc;
    };

    bool forward_blocked = false;
    for (const auto& obs : obstacles)
    {
        if (obs.y() > 0.0f && obs.y() < std::min(1700.0f, static_cast<float>(person_dist) + 200.0f) &&
            std::abs(obs.x()) < 420.0f)
        {
            forward_blocked = true;
            break;
        }
    }

    std::vector<Candidate> valid_candidates;
    valid_candidates.reserve(params_.adv_samples * params_.rot_samples + 1);

    const float adv_step = (params_.adv_samples > 1)
                               ? (params_.max_adv_speed - params_.min_adv_speed) / (params_.adv_samples - 1)
                               : 0.0f;
    const float rot_step = (params_.rot_samples > 1)
                               ? (2.0f * params_.max_rot_speed) / (params_.rot_samples - 1)
                               : 0.0f;

    float max_allowed_adv = params_.max_adv_speed;
    if (!forward_blocked)
    {
        if (person_dist < 800.0)
        {
            max_allowed_adv = 0.0f;
        }
        else if (person_dist < params_.target_dist_mm)
        {
            max_allowed_adv = std::clamp(prop_adv + 150.0f, 0.0f, params_.max_adv_speed * 0.6f);
        }
    }

    for (int i = 0; i < params_.adv_samples; ++i)
    {
        const float cand_adv = params_.min_adv_speed + i * adv_step;
        if (cand_adv > max_allowed_adv) continue;

        const float current_thresh = (cand_adv < 1.0f) ? params_.robot_radius_mm : collision_threshold;
        for (int j = 0; j < params_.rot_samples; ++j)
        {
            const float cand_rot = -params_.max_rot_speed + j * rot_step;

            const auto arc = GenerateArc(cand_adv, cand_rot, params_.sim_time_sec, params_.sim_steps);
            const auto [collision, min_obs_dist] = EvaluateObstacleClearance(arc, obstacles, current_thresh);

            if (!collision)
            {
                valid_candidates.push_back({cand_adv, cand_rot, 0.0f, min_obs_dist, arc});
            }
        }
    }

    // Add proportional command baseline candidate if collision-free
    {
        const auto prop_arc = GenerateArc(prop_adv, prop_rot, params_.sim_time_sec, params_.sim_steps);
        const auto [prop_collision, prop_min_dist] = EvaluateObstacleClearance(prop_arc, obstacles, collision_threshold);
        if (!prop_collision)
        {
            valid_candidates.push_back({prop_adv, prop_rot, 0.0f, prop_min_dist, prop_arc});
        }
    }

    float best_cost = std::numeric_limits<float>::max();
    Candidate best_candidate;
    bool found_candidate = false;

    for (auto& cand : valid_candidates)
    {
        const auto [end_x, end_y, end_theta] = ForwardModel(cand.adv, cand.rot, params_.sim_time_sec);

        const float dx_to_end = target_person_x - static_cast<float>(end_x);
        const float dy_to_end = target_person_y - static_cast<float>(end_y);
        const float dist_to_end = std::sqrt(dx_to_end * dx_to_end + dy_to_end * dy_to_end);

        const float cost_target_dist = std::abs(dist_to_end - params_.target_dist_mm) / params_.target_dist_mm;

        const double bearing_to_person = std::atan2(-dx_to_end, dy_to_end);
        const double heading_diff = std::abs(std::remainder(end_theta - bearing_to_person, 2.0 * M_PI));
        const float cost_heading = static_cast<float>(heading_diff / M_PI);
        const bool near_obstacle = forward_blocked || (cand.min_obs_dist < 700.0f);
        const float weight_heading_eff = near_obstacle ? 0.3f : params_.weight_heading;

        float cost_obstacle = 0.0f;
        if (cand.min_obs_dist < params_.obstacle_influence_dist_mm)
        {
            cost_obstacle = (params_.obstacle_influence_dist_mm - cand.min_obs_dist) /
                            (params_.obstacle_influence_dist_mm - collision_threshold);
            cost_obstacle = std::clamp(cost_obstacle, 0.0f, 2.0f);
        }

        float cost_speed = 0.0f;
        if (person_dist > 1400.0f || near_obstacle)
        {
            cost_speed = 1.0f - (cand.adv / params_.max_adv_speed);
        }

        float cost_dist_to_person = 0.0f;
        if (person_dist > 1600.0f)
        {
            cost_dist_to_person = dist_to_end / 4000.0f;
        }

        const float cost_prop = near_obstacle
                                    ? 0.0f
                                    : (0.5f * (std::abs(cand.adv - prop_adv) / params_.max_adv_speed) +
                                       0.5f * (std::abs(cand.rot - prop_rot) / (2.0f * params_.max_rot_speed)));

        float cost_hysteresis = 0.0f;
        if (std::abs(cand.rot) > 0.1f && std::abs(last_chosen_rot_) > 0.1f)
        {
            if (cand.rot * last_chosen_rot_ < 0.0f)
            {
                cost_hysteresis = 0.3f;
            }
        }

        float cost_corridor = 0.0f;
        if (forward_blocked)
        {
            if (std::abs(cand.rot) < 0.25f)
            {
                cost_corridor += 1.5f;
            }
            if (cand.adv < 100.0f)
            {
                cost_corridor += 0.8f;
            }
        }

        cand.cost = params_.weight_target_dist * cost_target_dist +
                    weight_heading_eff * cost_heading +
                    params_.weight_obstacle * cost_obstacle +
                    params_.weight_speed * cost_speed +
                    params_.weight_dist_to_person * cost_dist_to_person +
                    params_.weight_prop_alignment * cost_prop +
                    cost_hysteresis +
                    cost_corridor;

        if (cand.cost < best_cost)
        {
            best_cost = cand.cost;
            best_candidate = cand;
            found_candidate = true;
        }
    }

    if (found_candidate)
    {
        last_chosen_rot_ = best_candidate.rot;
        out.adv_speed = best_candidate.adv;
        out.rot_speed = best_candidate.rot;
        out.min_obs_dist = best_candidate.min_obs_dist;
        out.best_arc = best_candidate.arc;

        const bool diff_from_prop = (std::abs(out.adv_speed - prop_adv) > 100.0f ||
                                    std::abs(out.rot_speed - prop_rot) > 0.2f);
        out.is_avoiding = (out.min_obs_dist < params_.obstacle_influence_dist_mm) && diff_from_prop;
    }
    else
    {
        out.adv_speed = 0.0f;
        const double bearing_to_goal = std::atan2(-target_person_x, target_person_y);
        out.rot_speed = (bearing_to_goal >= 0.0) ? params_.max_rot_speed : -params_.max_rot_speed;
        out.is_avoiding = true;
        out.min_obs_dist = params_.robot_radius_mm;
        out.best_arc = GenerateArc(0.0, out.rot_speed, params_.sim_time_sec, params_.sim_steps);
    }

    for (size_t k = 0; k < valid_candidates.size(); k += std::max<size_t>(1, valid_candidates.size() / 15))
    {
        out.candidate_arcs.push_back(valid_candidates[k].arc);
    }

    return out;
}
