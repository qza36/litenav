#ifndef PLANNING_HPP
#define PLANNING_HPP

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core_types.hpp"

namespace planning_module
{
    struct GridCell
    {
        int x{0};
        int y{0};

        bool operator==(const GridCell &) const = default;
    };

    struct PlanResult
    {
        std::vector<GridCell> grid_path;
        std::vector<Pose2D> world_path;
        double cost{0.0};
    };

    class AStarPlanner
    {
    public:
        void setMap(std::shared_ptr<const GridMap> map);
        bool hasMap() const;

        void setAllowUnknown(bool allow_unknown);
        void setOccupiedThreshold(int8_t occupied_threshold);

        bool createPlan(const Pose2D &start, const Pose2D &goal, PlanResult &plan) const;
        bool createPlan(const GridCell &start, const GridCell &goal, PlanResult &plan) const;

        bool worldToGrid(const Pose2D &pose, GridCell &cell) const;
        bool gridToWorld(const GridCell &cell, Pose2D &pose) const;
        bool isTraversable(const GridCell &cell) const;
    private:
        bool isMapReady() const;
        bool isInBounds(const GridCell &cell) const;
        std::size_t toLinearIndex(const GridCell &cell) const;
        double heuristic(const GridCell &from, const GridCell &to) const;
        std::vector<GridCell> reconstructPath(
            int goal_index, const std::vector<int> &parents) const;
        std::vector<Pose2D> toWorldPath(const std::vector<GridCell> &grid_path) const;

        std::shared_ptr<const GridMap> map_;
        bool allow_unknown_{false};
        int8_t occupied_threshold_{50};
    };
}
#endif
