#ifndef NAVIGATION_APP_HPP
#define NAVIGATION_APP_HPP

#include <memory>
#include <string>

#include "core_types.hpp"
#include "pid_path_follower.hpp"
#include "planning.hpp"

namespace app_module
{
    struct NavigationAppConfig
    {
        std::string map_yaml;
        Pose2D start;
        Pose2D goal;
    };

    class NavigationApp
    {
    public:
        explicit NavigationApp(NavigationAppConfig config);

        const NavigationAppConfig & config() const;

        void setMapYaml(std::string map_yaml);
        void setStart(const Pose2D &start);
        void setGoal(const Pose2D &goal);

        bool loadMap();
        bool planPath();
        bool followPath();
        bool run();
        bool showDebugWindows() const;

        bool hasMap() const;
        bool hasPlan() const;
        bool hasTrajectory() const;

        const GridMap * map() const;
        const planning_module::PlanResult * plan() const;
        const std::vector<Pose2D> * trajectory() const;
        const std::string & lastError() const;
    private:
        void setError(std::string message);

        NavigationAppConfig config_;
        std::shared_ptr<GridMap> map_;
        planning_module::AStarPlanner planner_;
        planning_module::PlanResult plan_;
        control_module::PIDPathFollower follower_;
        std::vector<Pose2D> trajectory_;
        bool has_plan_{false};
        bool has_trajectory_{false};
        std::string last_error_;
    };
}

#endif
