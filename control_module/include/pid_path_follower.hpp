#ifndef PID_PATH_FOLLOWER_HPP
#define PID_PATH_FOLLOWER_HPP

#include <vector>

#include "core_types.hpp"

namespace control_module
{
    struct PIDConfig
    {
        double kp{1.0};
        double ki{0.0};
        double kd{0.3};
        double max_linear_vel{0.5};
        double max_angular_vel{1.0};
        double dt{0.01};
        double goal_tolerance{0.1};
        double lookahead_dist{0.3};
    };

    class PIDPathFollower
    {
    public:
        void setPath(const std::vector<Pose2D> &path);
        void setConfig(const PIDConfig &config);

        bool hasPath() const;
        const PIDConfig &config() const;

        bool followPath(const Pose2D &start, std::vector<Pose2D> &trajectory) const;

    private:
        struct ClosestResult
        {
            std::size_t seg_index{0};
            double t{0.0};
            double distance{0.0};
            double path_yaw{0.0};
            double cx{0.0};
            double cy{0.0};
        };

        ClosestResult findClosestSegment(double x, double y) const;
        static double normalizeAngle(double angle);
        static double distance2D(double x1, double y1, double x2, double y2);

        std::vector<Pose2D> path_;
        PIDConfig config_;
    };
}

#endif
