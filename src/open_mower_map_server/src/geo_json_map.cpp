#include "open_mower_map_server/geo_json_map.hpp"
#include <fstream>

#include <nlohmann/json.hpp>

using json = nlohmann::json;

namespace open_mower_map_server {
    GeoJSONMap::GeoJSONMap(std::string path) : _path(path) {
        if (_path.empty()) {
            throw std::invalid_argument("GeoJSON file path cannot be empty");
        }
    }

    open_mower_map_server::msg::Map::SharedPtr GeoJSONMap::load() {
        std::ifstream f(_path, std::ios::in);
        if (!f.is_open()) {
            throw std::runtime_error("Could not open GeoJSON file: " + _path);
        }

        json data = json::parse(f);
    }
} // OpenMowerMapServer