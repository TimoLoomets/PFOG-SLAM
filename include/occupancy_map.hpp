#pragma once
#include "occupancy_grid.hpp"
#include "ray.hpp"

namespace SLAM
{
    class OccupancyMap
    {
        std::array<std::shared_ptr<occupancy_grid_template>, 2> occupancy_grids;

        public:
        OccupancyMap()
        {
            occupancy_grids[0] = std::make_shared<OccupancyGrid<15>>();
            occupancy_grids[1] = std::make_shared<OccupancyGrid<5>>();
        }

        void add_detection(const Ray detection, const float confidence)
        {

        }
    };
}