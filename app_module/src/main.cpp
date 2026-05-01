#include <exception>
#include <iostream>
#include <string>

#include "navigation_app.hpp"

int main()
{
    const app_module::NavigationAppConfig config{
        "/Users/zhiangqi/CLionProjects/litenav/map_module/maps/test.yaml",
        {-1.05, -1.525, 0.0},
        {-0.975, 1.525, 0.0},
    };

    try {
        app_module::NavigationApp app(config);
        if (!app.run()) {
            std::cerr << "failed to run app: " << app.lastError() << std::endl;
            return 1;
        }

        const auto *plan = app.plan();
        if (plan == nullptr) {
            std::cerr << "plan is unavailable" << std::endl;
            return 1;
        }

        std::cout << "path found" << std::endl;
        std::cout << "  waypoints: " << plan->world_path.size() << std::endl;
        std::cout << "  length(m): " << plan->cost << std::endl;
        std::cout << "  start: (" << plan->world_path.front().x << ", "
                  << plan->world_path.front().y << ")" << std::endl;
        std::cout << "  goal: (" << plan->world_path.back().x << ", "
                  << plan->world_path.back().y << ")" << std::endl;

        const auto *traj = app.trajectory();
        if (traj == nullptr) {
            std::cerr << "trajectory is unavailable" << std::endl;
            return 1;
        }

        std::cout << "trajectory computed" << std::endl;
        std::cout << "  steps: " << traj->size() << std::endl;
        std::cout << "  end: (" << traj->back().x << ", "
                  << traj->back().y << ")" << std::endl;

        std::cout << "opening debug windows..." << std::endl;
        if (!app.showDebugWindows()) {
            std::cerr << "failed to open debug windows" << std::endl;
            return 1;
        }
    } catch (const std::exception &e) {
        std::cerr << "failed to run app module: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
