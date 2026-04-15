#include "map_io.hpp"

#include <cstring>
#include <filesystem>
#include <iostream>
#include <string>

#include "Magick++.h"
#include <eigen3/Eigen/Eigen>

map_module::LoadParameters
map_module::map_io::loadMapYaml(const std::string &yaml_filename)
{
    yaml_path_ = std::filesystem::absolute(yaml_filename);
    YAML::Node doc = YAML::LoadFile(yaml_filename);
    std::cout << "load yaml:" << yaml_path_ << "\n";

    params_.image_file_name = doc["image"].as<std::string>();
    params_.resolution = doc["resolution"].as<double>();
    params_.origin[0] = doc["origin"][0].as<double>();
    params_.origin[1] = doc["origin"][1].as<double>();
    params_.origin[2] = doc["origin"][2].as<double>();
    params_.negate = doc["negate"].as<int>() != 0;
    params_.occupied_thresh = doc["occupied_thresh"].as<double>();
    params_.free_thresh = doc["free_thresh"].as<double>();

    if (params_.image_file_name.empty()) {
        throw YAML::Exception(doc["image"].Mark(), "The image tag was empty.");
    }

    return params_;
}

map_module::map_io::LOAD_MAP_STATUS
map_module::map_io::loadMap()
{
    try {
        Magick::InitializeMagick(nullptr);

        std::filesystem::path image_path{params_.image_file_name};
        if (image_path.is_relative()) {
            image_path = yaml_path_.parent_path() / image_path;
        }

        Magick::Image img(image_path.lexically_normal().string());

        map_.width = img.columns();
        map_.height = img.rows();
        map_.resolution = params_.resolution;
        map_.origin.x = params_.origin[0];
        map_.origin.y = params_.origin[1];
        map_.origin.yaw = params_.origin[2];

        std::cout << "map info:\n"
                  << "width: " << map_.width << "\n"
                  << "height: " << map_.height << "\n";

        Magick::Image gray = img;
        gray.type(Magick::GrayscaleType);

        const std::size_t width = gray.columns();
        const std::size_t height = gray.rows();

        std::vector<uint8_t> buffer(width * height);
        gray.write(0, 0, width, height, "I", Magick::CharPixel, buffer.data());

        Eigen::Map<Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
            gray_matrix(buffer.data(), height, width);

        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            normalized = gray_matrix.cast<float>() / 255.0f;

        if (!params_.negate) {
            normalized = (1.0f - normalized.array()).matrix();
        }

        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> occupied =
            (normalized.array() > params_.occupied_thresh).cast<uint8_t>();

        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> free =
            (normalized.array() < params_.free_thresh).cast<uint8_t>();

        Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> result(height, width);
        result.setConstant(-1);
        result = (occupied.array() > 0).select(int8_t(100), result);
        result = (free.array() > 0).select(int8_t(0), result);

        Eigen::Matrix<int8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            flipped = result.colwise().reverse();

        map_.data.resize(width * height);
        std::memcpy(map_.data.data(), flipped.data(), width * height * sizeof(int8_t));

        return LOAD_MAP_SUCCESS;
    } catch (const Magick::Exception &) {
        return INVALID_MAP_DATA;
    } catch (const std::exception &) {
        return INVALID_MAP_DATA;
    }
}
