#include "pid_path_follower.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace control_module
{
    void PIDPathFollower::setPath(const std::vector<Pose2D> &path)
    {
        path_ = path;
    }

    void PIDPathFollower::setConfig(const PIDConfig &config)
    {
        config_ = config;
    }

    bool PIDPathFollower::hasPath() const
    {
        return path_.size() >= 2;
    }

    const PIDConfig &PIDPathFollower::config() const
    {
        return config_;
    }

    double PIDPathFollower::normalizeAngle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    double PIDPathFollower::distance2D(double x1, double y1, double x2, double y2)
    {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

    PIDPathFollower::ClosestResult PIDPathFollower::findClosestSegment(double x, double y) const
    {
        ClosestResult best;
        best.distance = std::numeric_limits<double>::max();

        for (std::size_t i = 0; i + 1 < path_.size(); ++i) {
            double ax = path_[i].x;
            double ay = path_[i].y;
            double bx = path_[i + 1].x;
            double by = path_[i + 1].y;

            double dx = bx - ax;
            double dy = by - ay;
            double len_sq = dx * dx + dy * dy;

            double t = 0.0;
            if (len_sq > 1e-12) {
                t = ((x - ax) * dx + (y - ay) * dy) / len_sq;
                t = std::clamp(t, 0.0, 1.0);
            }

            double cx = ax + t * dx;
            double cy = ay + t * dy;
            double dist = distance2D(x, y, cx, cy);

            if (dist < best.distance) {
                best.seg_index = i;
                best.t = t;
                best.distance = dist;
                best.path_yaw = std::atan2(dy, dx);
                best.cx = cx;
                best.cy = cy;
            }
        }

        return best;
    }

    bool PIDPathFollower::followPath(const Pose2D &start, std::vector<Pose2D> &trajectory) const
    {
        if (!hasPath()) {
            return false;
        }

        trajectory.clear();
        trajectory.reserve(static_cast<std::size_t>(10000));

        double x = start.x;
        double y = start.y;
        double yaw = start.yaw;

        const double goal_x = path_.back().x;
        const double goal_y = path_.back().y;
        const int max_steps = 100000;

        // Start from the closest segment
        auto closest = findClosestSegment(x, y);
        std::size_t seg_idx = closest.seg_index;

        for (int step = 0; step < max_steps; ++step) {
            trajectory.push_back({x, y, yaw});

            if (distance2D(x, y, goal_x, goal_y) < config_.goal_tolerance) {
                break;
            }

            // Find lookahead target along path from current segment
            double lx = goal_x;
            double ly = goal_y;
            double accum = 0.0;
            bool found = false;

            // Distance from robot to current segment's end
            double prev_x = path_[seg_idx].x;
            double prev_y = path_[seg_idx].y;
            double prev_dist = distance2D(x, y, prev_x, prev_y);

            for (std::size_t i = seg_idx; i + 1 < path_.size(); ++i) {
                double ax = path_[i].x;
                double ay = path_[i].y;
                double bx = path_[i + 1].x;
                double by = path_[i + 1].y;
                double seg_len = distance2D(ax, ay, bx, by);

                double robot_dist = distance2D(x, y, bx, by);
                double end_dist = std::min(robot_dist, prev_dist + seg_len);

                if (end_dist >= config_.lookahead_dist) {
                    // Interpolate along this segment
                    double frac = (config_.lookahead_dist - prev_dist) / (end_dist - prev_dist);
                    frac = std::clamp(frac, 0.0, 1.0);
                    lx = ax + frac * (bx - ax);
                    ly = ay + frac * (by - ay);
                    found = true;
                    break;
                }

                prev_dist = end_dist;
            }

            if (!found) {
                lx = goal_x;
                ly = goal_y;
            }

            // Steer toward lookahead point
            double target_yaw = std::atan2(ly - y, lx - x);
            double heading_err = normalizeAngle(target_yaw - yaw);

            // Reduce speed when heading is off
            double abs_err = std::abs(heading_err);
            double v_scale = std::max(0.2, std::cos(heading_err));
            double v = config_.max_linear_vel * v_scale;

            double omega = config_.kp * heading_err;
            omega = std::clamp(omega, -config_.max_angular_vel, config_.max_angular_vel);

            x += v * std::cos(yaw) * config_.dt;
            y += v * std::sin(yaw) * config_.dt;
            yaw = normalizeAngle(yaw + omega * config_.dt);

            // Advance segment index when robot passes the current segment end
            if (seg_idx + 1 < path_.size()) {
                double ex = path_[seg_idx + 1].x;
                double ey = path_[seg_idx + 1].y;
                double seg_dx = ex - path_[seg_idx].x;
                double seg_dy = ey - path_[seg_idx].y;
                double to_robot_dx = x - path_[seg_idx].x;
                double to_robot_dy = y - path_[seg_idx].y;
                if (to_robot_dx * seg_dx + to_robot_dy * seg_dy >=
                    (seg_dx * seg_dx + seg_dy * seg_dy) * 0.95) {
                    ++seg_idx;
                }
            }
        }

        return true;
    }
}
