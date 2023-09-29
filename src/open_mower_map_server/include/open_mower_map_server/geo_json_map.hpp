#pragma once

#include "map_server_node.hpp"

namespace open_mower_map_server {
    class GeoJSONMap : public MapIO {
    public:
        explicit GeoJSONMap(std::string path);

        void save(open_mower_map_server::msg::Map::SharedPtr map);
        open_mower_map_server::msg::Map::SharedPtr load();

    private:
        std::string _path;
    };

} // OpenMowerMapServer
