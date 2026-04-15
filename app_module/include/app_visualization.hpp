#ifndef APP_VISUALIZATION_HPP
#define APP_VISUALIZATION_HPP

#include <string>
#include <vector>

#include "core_types.hpp"
#include "planning.hpp"

namespace app_module
{
    bool renderMapImage(
        const GridMap &map,
        std::vector<uint8_t> &image,
        bool draw_world_axes = true);

    bool renderPlanImage(
        const GridMap &map,
        const planning_module::PlanResult &plan,
        std::vector<uint8_t> &image);

    bool showMapWindow(
        const GridMap &map,
        const std::string &window_title = "Map With Scale And Center");

    bool showPlanWindow(
        const GridMap &map,
        const planning_module::PlanResult &plan,
        const std::string &window_title = "Planned Route");

    bool showDebugWindows(const GridMap &map, const planning_module::PlanResult &plan);
}

#endif
