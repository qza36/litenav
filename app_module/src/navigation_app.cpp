#include "navigation_app.hpp"

#include <utility>

#include "app_visualization.hpp"
#include "map_io.hpp"

namespace app_module
{
NavigationApp::NavigationApp(NavigationAppConfig config)
    : config_(std::move(config))
{
}

const NavigationAppConfig & NavigationApp::config() const
{
    return config_;
}

void NavigationApp::setMapYaml(std::string map_yaml)
{
    config_.map_yaml = std::move(map_yaml);
}

void NavigationApp::setStart(const Pose2D &start)
{
    config_.start = start;
}

void NavigationApp::setGoal(const Pose2D &goal)
{
    config_.goal = goal;
}

bool NavigationApp::loadMap()
{
    map_ = nullptr;
    has_plan_ = false;
    plan_ = {};
    last_error_.clear();

    try {
        map_module::map_io map_loader(config_.map_yaml);
        const auto status = map_loader.loadMap();
        if (status != map_module::map_io::LOAD_MAP_SUCCESS) {
            setError("failed to load map");
            return false;
        }

        map_ = std::make_shared<GridMap>(map_loader.getMap());
        planner_.setMap(map_);
        return true;
    } catch (const std::exception &e) {
        setError(e.what());
        return false;
    }
}

bool NavigationApp::planPath()
{
    has_plan_ = false;
    plan_ = {};
    last_error_.clear();

    if (!map_) {
        setError("map is not loaded");
        return false;
    }

    if (!planner_.createPlan(config_.start, config_.goal, plan_)) {
        setError("failed to create a path");
        return false;
    }

    has_plan_ = true;
    return true;
}

bool NavigationApp::followPath()
{
    has_trajectory_ = false;
    trajectory_.clear();
    last_error_.clear();

    if (!has_plan_) {
        setError("no plan available");
        return false;
    }

    if (!follower_.hasPath()) {
        follower_.setPath(plan_.world_path);
    }

    if (!follower_.followPath(config_.start, trajectory_)) {
        setError("failed to follow path");
        return false;
    }

    has_trajectory_ = true;
    return true;
}

bool NavigationApp::run()
{
    return loadMap() && planPath() && followPath();
}

bool NavigationApp::showDebugWindows() const
{
    if (!map_ || !has_plan_) {
        return false;
    }

    if (has_trajectory_) {
        return showTrajectoryWindow(*map_, plan_, trajectory_);
    }

    return app_module::showDebugWindows(*map_, plan_);
}

bool NavigationApp::hasMap() const
{
    return map_ != nullptr;
}

bool NavigationApp::hasPlan() const
{
    return has_plan_;
}

bool NavigationApp::hasTrajectory() const
{
    return has_trajectory_;
}

const GridMap * NavigationApp::map() const
{
    return map_.get();
}

const planning_module::PlanResult * NavigationApp::plan() const
{
    return has_plan_ ? &plan_ : nullptr;
}

const std::vector<Pose2D> * NavigationApp::trajectory() const
{
    return has_trajectory_ ? &trajectory_ : nullptr;
}

const std::string & NavigationApp::lastError() const
{
    return last_error_;
}

void NavigationApp::setError(std::string message)
{
    last_error_ = std::move(message);
}
}  // namespace app_module
