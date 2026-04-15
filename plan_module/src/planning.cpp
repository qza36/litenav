#include "planning.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace planning_module
{
namespace
{
struct QueueNode
{
    int index{0};
    double f_cost{0.0};
};

struct QueueCompare
{
    bool operator()(const QueueNode &lhs, const QueueNode &rhs) const
    {
        return lhs.f_cost > rhs.f_cost;
    }
};

constexpr std::array<GridCell, 8> kNeighborOffsets{{
    {1, 0},
    {-1, 0},
    {0, 1},
    {0, -1},
    {1, 1},
    {1, -1},
    {-1, 1},
    {-1, -1},
}};
}  // namespace

void AStarPlanner::setMap(std::shared_ptr<const GridMap> map)
{
    map_ = std::move(map);
}

bool AStarPlanner::hasMap() const
{
    return isMapReady();
}

void AStarPlanner::setAllowUnknown(bool allow_unknown)
{
    allow_unknown_ = allow_unknown;
}

void AStarPlanner::setOccupiedThreshold(int8_t occupied_threshold)
{
    occupied_threshold_ = std::clamp<int>(occupied_threshold, 0, 100);
}

bool AStarPlanner::createPlan(const Pose2D &start, const Pose2D &goal, PlanResult &plan) const
{
    plan = {};

    GridCell start_cell;
    GridCell goal_cell;
    if (!worldToGrid(start, start_cell) || !worldToGrid(goal, goal_cell)) {
        return false;
    }

    if (!createPlan(start_cell, goal_cell, plan)) {
        return false;
    }

    if (plan.world_path.empty()) {
        plan = {};
        return false;
    }

    plan.world_path.front().yaw = start.yaw;
    plan.world_path.back().yaw = goal.yaw;
    return true;
}

bool AStarPlanner::createPlan(const GridCell &start, const GridCell &goal, PlanResult &plan) const
{
    plan = {};

    if (!isMapReady() || !isTraversable(start) || !isTraversable(goal)) {
        return false;
    }

    const int width = static_cast<int>(map_->width);
    const auto map_size = static_cast<std::size_t>(map_->width) * map_->height;
    const auto start_index = static_cast<int>(toLinearIndex(start));
    const auto goal_index = static_cast<int>(toLinearIndex(goal));
    const double infinity = std::numeric_limits<double>::infinity();

    std::priority_queue<QueueNode, std::vector<QueueNode>, QueueCompare> open_set;
    std::vector<double> g_score(map_size, infinity);
    std::vector<int> parents(map_size, -1);
    std::vector<bool> closed(map_size, false);

    g_score[start_index] = 0.0;
    open_set.push({start_index, heuristic(start, goal)});

    while (!open_set.empty()) {
        const auto current = open_set.top();
        open_set.pop();

        if (closed[static_cast<std::size_t>(current.index)]) {
            continue;
        }

        if (current.index == goal_index) {
            plan.grid_path = reconstructPath(goal_index, parents);
            plan.world_path = toWorldPath(plan.grid_path);
            plan.cost = g_score[goal_index];
            return true;
        }

        closed[static_cast<std::size_t>(current.index)] = true;
        const GridCell current_cell{
            current.index % width,
            current.index / width,
        };

        for (const auto &offset : kNeighborOffsets) {
            const GridCell next_cell{
                current_cell.x + offset.x,
                current_cell.y + offset.y,
            };

            if (!isTraversable(next_cell)) {
                continue;
            }

            const bool is_diagonal = offset.x != 0 && offset.y != 0;
            if (is_diagonal) {
                const GridCell side_a{current_cell.x + offset.x, current_cell.y};
                const GridCell side_b{current_cell.x, current_cell.y + offset.y};
                if (!isTraversable(side_a) || !isTraversable(side_b)) {
                    continue;
                }
            }

            const auto next_index = static_cast<int>(toLinearIndex(next_cell));
            if (closed[static_cast<std::size_t>(next_index)]) {
                continue;
            }

            const double step_cost =
                std::hypot(static_cast<double>(offset.x), static_cast<double>(offset.y)) *
                static_cast<double>(map_->resolution);
            const double tentative_g = g_score[static_cast<std::size_t>(current.index)] + step_cost;

            if (tentative_g >= g_score[static_cast<std::size_t>(next_index)]) {
                continue;
            }

            parents[static_cast<std::size_t>(next_index)] = current.index;
            g_score[static_cast<std::size_t>(next_index)] = tentative_g;
            open_set.push({next_index, tentative_g + heuristic(next_cell, goal)});
        }
    }

    return false;
}

bool AStarPlanner::worldToGrid(const Pose2D &pose, GridCell &cell) const
{
    cell = {};

    if (!isMapReady()) {
        return false;
    }

    const double dx = pose.x - map_->origin.x;
    const double dy = pose.y - map_->origin.y;
    const double cos_yaw = std::cos(map_->origin.yaw);
    const double sin_yaw = std::sin(map_->origin.yaw);

    const double local_x = cos_yaw * dx + sin_yaw * dy;
    const double local_y = -sin_yaw * dx + cos_yaw * dy;
    cell = {
        static_cast<int>(std::floor(local_x / map_->resolution)),
        static_cast<int>(std::floor(local_y / map_->resolution)),
    };

    if (!isInBounds(cell)) {
        cell = {};
        return false;
    }

    return true;
}

bool AStarPlanner::gridToWorld(const GridCell &cell, Pose2D &pose) const
{
    pose = {};

    if (!isInBounds(cell)) {
        return false;
    }

    const double local_x = (static_cast<double>(cell.x) + 0.5) * map_->resolution;
    const double local_y = (static_cast<double>(cell.y) + 0.5) * map_->resolution;
    const double cos_yaw = std::cos(map_->origin.yaw);
    const double sin_yaw = std::sin(map_->origin.yaw);

    pose.x = map_->origin.x + cos_yaw * local_x - sin_yaw * local_y;
    pose.y = map_->origin.y + sin_yaw * local_x + cos_yaw * local_y;
    pose.yaw = map_->origin.yaw;
    return true;
}

bool AStarPlanner::isTraversable(const GridCell &cell) const
{
    if (!isInBounds(cell)) {
        return false;
    }

    const auto value = map_->data[toLinearIndex(cell)];
    if (value < 0) {
        return allow_unknown_;
    }

    return value < occupied_threshold_;
}

bool AStarPlanner::isMapReady() const
{
    return map_ != nullptr &&
           map_->width > 0 &&
           map_->height > 0 &&
           map_->resolution > 0.0f &&
           map_->data.size() == static_cast<std::size_t>(map_->width) * map_->height;
}

bool AStarPlanner::isInBounds(const GridCell &cell) const
{
    if (!isMapReady()) {
        return false;
    }

    return cell.x >= 0 &&
           cell.y >= 0 &&
           static_cast<uint32_t>(cell.x) < map_->width &&
           static_cast<uint32_t>(cell.y) < map_->height;
}

std::size_t AStarPlanner::toLinearIndex(const GridCell &cell) const
{
    return static_cast<std::size_t>(cell.y) * map_->width + static_cast<std::size_t>(cell.x);
}

double AStarPlanner::heuristic(const GridCell &from, const GridCell &to) const
{
    return std::hypot(
               static_cast<double>(to.x - from.x),
               static_cast<double>(to.y - from.y)) *
           static_cast<double>(map_->resolution);
}

std::vector<GridCell> AStarPlanner::reconstructPath(
    int goal_index, const std::vector<int> &parents) const
{
    std::vector<GridCell> path;
    const int width = static_cast<int>(map_->width);

    for (int current_index = goal_index; current_index >= 0;
         current_index = parents[static_cast<std::size_t>(current_index)]) {
        path.push_back({
            current_index % width,
            current_index / width,
        });
    }

    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Pose2D> AStarPlanner::toWorldPath(const std::vector<GridCell> &grid_path) const
{
    std::vector<Pose2D> world_path;
    world_path.reserve(grid_path.size());

    for (const auto &cell : grid_path) {
        Pose2D pose;
        if (gridToWorld(cell, pose)) {
            world_path.push_back(pose);
        }
    }

    for (std::size_t i = 0; i + 1 < world_path.size(); ++i) {
        const auto dx = world_path[i + 1].x - world_path[i].x;
        const auto dy = world_path[i + 1].y - world_path[i].y;
        world_path[i].yaw = std::atan2(dy, dx);
    }

    return world_path;
}
}  // namespace planning_module
