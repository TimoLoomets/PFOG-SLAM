#pragma once
#include <memory>
#include <array>
#include <math.h>

#include "occupancy_grid_template.hpp"

namespace SLAM
{
    // Configuration constants
    constexpr float GRID_SIZE = 0.05f;

    template<int MAP_SIZE>
    class OccupancyGrid: public occupancy_grid_template
    {
        constexpr static size_t MAP_CELLS = static_cast<size_t>(static_cast<float>(MAP_SIZE) / GRID_SIZE);
        constexpr static float OFFSET = static_cast<float>(MAP_SIZE) / 2.0f;

        std::array<std::array<float, MAP_CELLS>, MAP_CELLS> grid;

        public:
        OccupancyGrid()
        {
            for (uint i = 0; i < MAP_CELLS; i++)
            {
                grid[i].fill(0.5f);
            }
        }

        float get_occupancy(const float x, const float y) const override
        {
            const float offset_x = x + OFFSET;
            const float offset_y = y + OFFSET;

            const int cell_x = static_cast<int>(offset_x / GRID_SIZE);
            if (cell_x < 0 || cell_x > MAP_CELLS - 1)
            {
                return 0.5f;
            }
            
            const int cell_y = static_cast<int>(offset_y / GRID_SIZE);
            if (cell_y < 0 || cell_y > MAP_CELLS - 1)
            {
                return 0.5f;
            }

            return grid[cell_x][cell_y];
        }

        void set_occupancy(const float x, const float y, const float occupancy) override
        {
            const float offset_x = x + OFFSET;
            const float offset_y = y + OFFSET;

            const int cell_x = static_cast<int>(offset_x / GRID_SIZE);
            if (cell_x < 0 || cell_x > MAP_CELLS - 1)
            {
                return;
            }
            
            const int cell_y = static_cast<int>(offset_y / GRID_SIZE);
            if (cell_y < 0 || cell_y > MAP_CELLS - 1)
            {
                return;
            }

            grid[cell_x][cell_y] = occupancy;
        }
    };
}