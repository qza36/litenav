#ifndef MAP_IO_HPP
#define MAP_IO_HPP

#include <filesystem>
#include <string>
#include <vector>

#include "core_types.hpp"
#include "yaml-cpp/yaml.h"

namespace map_module {
    struct LoadParameters {
        std::string image_file_name;
        double resolution{0};
        std::vector<double> origin{0, 0, 0};
        double free_thresh{0.196};
        double occupied_thresh{0.65};
        bool negate{false};
    };
    class map_io {
    public:
        typedef enum
        {
            LOAD_MAP_SUCCESS,
            MAP_DOES_NOT_EXIST,
            INVALID_MAP_METADATA,
            INVALID_MAP_DATA
          } LOAD_MAP_STATUS;
        explicit map_io(const std::string &yaml_filename) {
            loadMapYaml(yaml_filename);
        }
        ~map_io() {

        }
        /**
         * @brief Load and parse the given YAML file
         * @param yaml_filename Name of the map file passed though parameter
         * @return Map loading parameters obtained from YAML file
         * @throw YAML::Exception
         */
        LoadParameters loadMapYaml(const std::string & yaml_filename);

        /**
         * @brief Load the map image from map file and
         * generate an OccupancyGrid
         * @return status of map loaded
         */
        LOAD_MAP_STATUS loadMap();
        /**
         * @brief Access loaded map data.
         */
        const GridMap & getMap() const {
            return map_;
        }
    private:
        GridMap map_;
        LoadParameters params_;
        std::filesystem::path yaml_path_;

    };

}

#endif
