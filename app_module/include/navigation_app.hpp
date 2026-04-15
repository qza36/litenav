#ifndef NAVIGATION_APP_HPP
#define NAVIGATION_APP_HPP

#include <memory>
#include <string>

#include "core_types.hpp"
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
        bool run();
        bool showDebugWindows() const;

        bool hasMap() const;
        bool hasPlan() const;

        const GridMap * map() const;
        const planning_module::PlanResult * plan() const;
        const std::string & lastError() const;
    private:
        void setError(std::string message);

        NavigationAppConfig config_;
        std::shared_ptr<GridMap> map_;
        planning_module::AStarPlanner planner_;
        planning_module::PlanResult plan_;
        bool has_plan_{false};
        std::string last_error_;
    };
}

#endif
