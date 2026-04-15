#ifndef CORE_TYPES_HPP
#define CORE_TYPES_HPP

#include <cstdint>
#include <vector>

struct Pose2D {
    double x{0.0};
    double y{0.0};
    double yaw{0.0};
};

struct GridMap {
    uint32_t width{0};
    uint32_t height{0};
    float resolution{0.0f};
    Pose2D origin;
    std::vector<int8_t> data;
};

#endif
