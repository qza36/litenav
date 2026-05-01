#include "app_visualization.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "Magick++.h"

namespace app_module
{
namespace
{
struct RgbPixel
{
    uint8_t r{0};
    uint8_t g{0};
    uint8_t b{0};
};

int toImageY(const GridMap &map, int grid_y)
{
    return static_cast<int>(map.height) - 1 - grid_y;
}

void setPixel(
    std::vector<uint8_t> &image, uint32_t width, uint32_t height, int x, int y, const RgbPixel &color)
{
    if (x < 0 || y < 0 || x >= static_cast<int>(width) || y >= static_cast<int>(height)) {
        return;
    }

    const auto index =
        (static_cast<std::size_t>(y) * width + static_cast<std::size_t>(x)) * 3;
    image[index] = color.r;
    image[index + 1] = color.g;
    image[index + 2] = color.b;
}

void paintSquare(
    std::vector<uint8_t> &image,
    const GridMap &map,
    int grid_x,
    int grid_y,
    int radius,
    const RgbPixel &color)
{
    const int image_y = toImageY(map, grid_y);
    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            setPixel(image, map.width, map.height, grid_x + dx, image_y + dy, color);
        }
    }
}

void paintCell(
    std::vector<uint8_t> &image,
    const GridMap &map,
    const planning_module::GridCell &cell,
    int radius,
    const RgbPixel &color)
{
    paintSquare(image, map, cell.x, cell.y, radius, color);
}

void drawLine(
    std::vector<uint8_t> &image,
    const GridMap &map,
    int x0,
    int y0,
    int x1,
    int y1,
    int thickness,
    const RgbPixel &color)
{
    int dx = std::abs(x1 - x0);
    int sx = x0 < x1 ? 1 : -1;
    int dy = -std::abs(y1 - y0);
    int sy = y0 < y1 ? 1 : -1;
    int error = dx + dy;

    while (true) {
        paintSquare(image, map, x0, y0, thickness, color);
        if (x0 == x1 && y0 == y1) {
            break;
        }

        const int twice_error = 2 * error;
        if (twice_error >= dy) {
            error += dy;
            x0 += sx;
        }
        if (twice_error <= dx) {
            error += dx;
            y0 += sy;
        }
    }
}

bool worldToGrid(const GridMap &map, double world_x, double world_y, int &grid_x, int &grid_y)
{
    if (map.width == 0 || map.height == 0 || map.resolution <= 0.0f) {
        return false;
    }

    const double dx = world_x - map.origin.x;
    const double dy = world_y - map.origin.y;
    const double cos_yaw = std::cos(map.origin.yaw);
    const double sin_yaw = std::sin(map.origin.yaw);
    const double local_x = cos_yaw * dx + sin_yaw * dy;
    const double local_y = -sin_yaw * dx + cos_yaw * dy;

    grid_x = static_cast<int>(std::floor(local_x / map.resolution));
    grid_y = static_cast<int>(std::floor(local_y / map.resolution));

    return grid_x >= 0 &&
           grid_y >= 0 &&
           static_cast<uint32_t>(grid_x) < map.width &&
           static_cast<uint32_t>(grid_y) < map.height;
}

void drawWorldAxes(std::vector<uint8_t> &image, const GridMap &map)
{
    int origin_x = 0;
    int origin_y = 0;
    if (!worldToGrid(map, 0.0, 0.0, origin_x, origin_y)) {
        return;
    }

    const double axis_length_m = 1.0;
    const double axis_length_cell = axis_length_m / map.resolution;
    const double cos_yaw = std::cos(map.origin.yaw);
    const double sin_yaw = std::sin(map.origin.yaw);

    const int x_axis_end_x =
        static_cast<int>(std::lround(static_cast<double>(origin_x) + axis_length_cell * cos_yaw));
    const int x_axis_end_y =
        static_cast<int>(std::lround(static_cast<double>(origin_y) - axis_length_cell * sin_yaw));

    const int y_axis_end_x =
        static_cast<int>(std::lround(static_cast<double>(origin_x) + axis_length_cell * sin_yaw));
    const int y_axis_end_y =
        static_cast<int>(std::lround(static_cast<double>(origin_y) + axis_length_cell * cos_yaw));

    drawLine(image, map, origin_x, origin_y, x_axis_end_x, x_axis_end_y, 1, {255, 0, 0});
    drawLine(image, map, origin_x, origin_y, y_axis_end_x, y_axis_end_y, 1, {0, 200, 0});
    paintSquare(image, map, origin_x, origin_y, 2, {255, 215, 0});
}

double chooseScaleBarLengthMeters(const GridMap &map)
{
    const double map_width_m = static_cast<double>(map.width) * map.resolution;
    const double target = std::max(0.25, map_width_m * 0.2);
    const double magnitude = std::pow(10.0, std::floor(std::log10(target)));

    double best = magnitude;
    for (const double factor : {1.0, 2.0, 5.0, 10.0}) {
        const double candidate = factor * magnitude;
        if (candidate <= target) {
            best = candidate;
        }
    }

    return best;
}

std::string formatMeters(double value)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(value < 1.0 ? 2 : 1) << value << " m";
    return stream.str();
}

std::string formatCenterCoordinates(const GridMap &map)
{
    const double local_center_x = static_cast<double>(map.width) * map.resolution * 0.5;
    const double local_center_y = static_cast<double>(map.height) * map.resolution * 0.5;
    const double cos_yaw = std::cos(map.origin.yaw);
    const double sin_yaw = std::sin(map.origin.yaw);

    const double world_center_x =
        map.origin.x + cos_yaw * local_center_x - sin_yaw * local_center_y;
    const double world_center_y =
        map.origin.y + sin_yaw * local_center_x + cos_yaw * local_center_y;

    std::ostringstream stream;
    stream << std::fixed << std::setprecision(2)
           << "center: (" << world_center_x << ", " << world_center_y << ")";
    return stream.str();
}

std::string findUsableFontPath()
{
    constexpr const char *candidates[] = {
        "/usr/share/fonts/truetype/noto/NotoSans-Regular.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf",
        "/usr/share/fonts/truetype/liberation/LiberationSans-Regular.ttf",
    };

    for (const auto *candidate : candidates) {
        if (std::filesystem::exists(candidate)) {
            return candidate;
        }
    }

    return {};
}

bool createMagickImage(
    const std::vector<uint8_t> &image,
    uint32_t width,
    uint32_t height,
    const std::string &window_title,
    Magick::Image &magick_image)
{
    if (image.size() != static_cast<std::size_t>(width) * height * 3) {
        return false;
    }

    magick_image.read(width, height, "RGB", Magick::CharPixel, image.data());
    magick_image.fileName(window_title);
    magick_image.label(window_title);
    return true;
}

void addMapFooter(Magick::Image &image, const GridMap &map)
{
    const std::size_t footer_height = 80;
    image.extent(
        Magick::Geometry(map.width, map.height + footer_height),
        Magick::Color("white"),
        Magick::NorthWestGravity);

    Magick::DrawableList graphics;
    graphics.push_back(Magick::DrawableStrokeColor(Magick::Color("black")));
    graphics.push_back(Magick::DrawableFillColor(Magick::Color("black")));
    graphics.push_back(Magick::DrawableStrokeWidth(2.0));

    const double scale_bar_m = chooseScaleBarLengthMeters(map);
    const double scale_bar_px = scale_bar_m / map.resolution;
    const double bar_x0 = 20.0;
    const double bar_y = static_cast<double>(map.height) + 54.0;
    const double bar_x1 = bar_x0 + scale_bar_px;

    graphics.push_back(Magick::DrawableLine(bar_x0, bar_y, bar_x1, bar_y));
    graphics.push_back(Magick::DrawableLine(bar_x0, bar_y - 6.0, bar_x0, bar_y + 6.0));
    graphics.push_back(Magick::DrawableLine(bar_x1, bar_y - 6.0, bar_x1, bar_y + 6.0));
    image.draw(graphics);

    const std::string font_path = findUsableFontPath();
    if (font_path.empty()) {
        return;
    }

    try {
        image.font(font_path);
        image.fontPointsize(16.0);

        Magick::DrawableList text;
        text.push_back(Magick::DrawableStrokeColor(Magick::Color("black")));
        text.push_back(Magick::DrawableFillColor(Magick::Color("black")));
        text.push_back(Magick::DrawableTextAntialias(true));
        text.push_back(Magick::DrawableText(
            20.0,
            static_cast<double>(map.height) + 24.0,
            formatCenterCoordinates(map)));
        text.push_back(Magick::DrawableText(bar_x0, bar_y + 22.0, formatMeters(scale_bar_m)));
        image.draw(text);
    } catch (const Magick::Exception &) {
    }
}

bool displayImageWindow(Magick::Image image)
{
    const pid_t child = fork();
    if (child < 0) {
        return false;
    }

    if (child == 0) {
        try {
            image.display();
            std::_Exit(EXIT_SUCCESS);
        } catch (const std::exception &e) {
            std::cerr << "failed to display image window: " << e.what() << std::endl;
            std::_Exit(EXIT_FAILURE);
        }
    }

    int status = 0;
    if (waitpid(child, &status, 0) < 0) {
        return false;
    }

    return WIFEXITED(status) && WEXITSTATUS(status) == EXIT_SUCCESS;
}
}  // namespace

bool renderMapImage(const GridMap &map, std::vector<uint8_t> &image, bool draw_world_axes)
{
    if (map.width == 0 ||
        map.height == 0 ||
        map.data.size() != static_cast<std::size_t>(map.width) * map.height) {
        image.clear();
        return false;
    }

    image.assign(static_cast<std::size_t>(map.width) * map.height * 3, 0);

    for (uint32_t y = 0; y < map.height; ++y) {
        for (uint32_t x = 0; x < map.width; ++x) {
            const auto value = map.data[static_cast<std::size_t>(y) * map.width + x];

            RgbPixel color{180, 180, 180};
            if (value == 0) {
                color = {255, 255, 255};
            } else if (value > 0) {
                color = {0, 0, 0};
            }

            setPixel(
                image,
                map.width,
                map.height,
                static_cast<int>(x),
                toImageY(map, static_cast<int>(y)),
                color);
        }
    }

    if (draw_world_axes) {
        drawWorldAxes(image, map);
    }

    return true;
}

bool renderPlanImage(
    const GridMap &map,
    const planning_module::PlanResult &plan,
    std::vector<uint8_t> &image)
{
    if (plan.grid_path.empty()) {
        image.clear();
        return false;
    }

    if (!renderMapImage(map, image, true)) {
        return false;
    }

    for (const auto &cell : plan.grid_path) {
        paintCell(image, map, cell, 1, {255, 0, 0});
    }

    paintCell(image, map, plan.grid_path.front(), 3, {0, 220, 0});
    paintCell(image, map, plan.grid_path.back(), 3, {0, 80, 255});
    return true;
}

bool showMapWindow(const GridMap &map, const std::string &window_title)
{
    std::vector<uint8_t> image;
    if (!renderMapImage(map, image, true)) {
        return false;
    }

    Magick::Image magick_image;
    if (!createMagickImage(image, map.width, map.height, window_title, magick_image)) {
        return false;
    }

    addMapFooter(magick_image, map);
    return displayImageWindow(magick_image);
}

bool showPlanWindow(
    const GridMap &map,
    const planning_module::PlanResult &plan,
    const std::string &window_title)
{
    std::vector<uint8_t> image;
    if (!renderPlanImage(map, plan, image)) {
        return false;
    }

    Magick::Image magick_image;
    if (!createMagickImage(image, map.width, map.height, window_title, magick_image)) {
        return false;
    }

    return displayImageWindow(magick_image);
}

bool showDebugWindows(const GridMap &map, const planning_module::PlanResult &plan)
{
    bool map_window_ok = false;
    bool route_window_ok = false;

    std::thread map_thread([&]() {
        map_window_ok = showMapWindow(map);
    });

    std::thread route_thread([&]() {
        route_window_ok = showPlanWindow(map, plan);
    });

    map_thread.join();
    route_thread.join();
    return map_window_ok && route_window_ok;
}

bool renderTrajectoryImage(
    const GridMap &map,
    const planning_module::PlanResult &plan,
    const std::vector<Pose2D> &trajectory,
    std::vector<uint8_t> &image)
{
    if (!renderPlanImage(map, plan, image)) {
        return false;
    }

    for (const auto &pose : trajectory) {
        int gx = 0;
        int gy = 0;
        if (worldToGrid(map, pose.x, pose.y, gx, gy)) {
            paintSquare(image, map, gx, gy, 1, {0, 200, 0});
        }
    }

    if (!trajectory.empty()) {
        int sx = 0;
        int sy = 0;
        if (worldToGrid(map, trajectory.front().x, trajectory.front().y, sx, sy)) {
            paintSquare(image, map, sx, sy, 3, {0, 0, 255});
        }
        int ex = 0;
        int ey = 0;
        if (worldToGrid(map, trajectory.back().x, trajectory.back().y, ex, ey)) {
            paintSquare(image, map, ex, ey, 3, {255, 165, 0});
        }
    }

    return true;
}

bool showTrajectoryWindow(
    const GridMap &map,
    const planning_module::PlanResult &plan,
    const std::vector<Pose2D> &trajectory,
    const std::string &window_title)
{
    std::vector<uint8_t> image;
    if (!renderTrajectoryImage(map, plan, trajectory, image)) {
        return false;
    }

    Magick::Image magick_image;
    if (!createMagickImage(image, map.width, map.height, window_title, magick_image)) {
        return false;
    }

    return displayImageWindow(magick_image);
}
}  // namespace app_module
