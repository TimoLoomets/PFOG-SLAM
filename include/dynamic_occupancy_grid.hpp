#pragma once
#include <memory>
#include <array>
#include <math.h>


namespace SLAM
{
    // Configuration constants
    constexpr float MAP_SIZE_X = 15.0f;
    constexpr float MAP_SIZE_Y = 15.0f;
    constexpr float CHUNK_SIZE = 1.0f;
    constexpr float GRID_SIZE = 0.05f;

    constexpr int MAX_CELL_VALUE = 255;

    // Derived constants
    constexpr size_t MAP_CHUNKS_X = static_cast<size_t>(MAP_SIZE_X / CHUNK_SIZE);
    constexpr size_t MAP_CHUNKS_Y = static_cast<size_t>(MAP_SIZE_Y / CHUNK_SIZE);
    constexpr size_t CHUNK_CELLS = static_cast<size_t>(CHUNK_SIZE / GRID_SIZE);
    constexpr float X_OFFSET = MAP_SIZE_X / 2.0f;
    constexpr float Y_OFFSET = MAP_SIZE_Y / 2.0f;

    typedef std::array<std::array<uint, CHUNK_CELLS>, CHUNK_CELLS> chunk_t; // TODO: Perhaps this could be changed to uint8_t

    class DynamicOccupancyGrid
    {
        std::array<std::array<std::shared_ptr<chunk_t>, MAP_CHUNKS_Y>, MAP_CHUNKS_X> grid;

        public:
        float get_occupancy(const float x, const float y) const
        {
            const float offset_x = x + X_OFFSET;
            const float offset_y = y + Y_OFFSET;

            const int chunk_x = static_cast<int>(offset_x / CHUNK_SIZE);
            if (chunk_x < 0 || chunk_x > MAP_CHUNKS_X - 1)
            {
                return 0.5f;
            }
            
            const int chunk_y = static_cast<int>(offset_y / CHUNK_SIZE);
            if (chunk_y < 0 || chunk_y > MAP_CHUNKS_Y - 1)
            {
                return 0.5f;
            }

            if (!grid[chunk_x][chunk_y])
            {
                return 0.5f;
            }

            const int cell_x = static_cast<int>(fmodf(offset_x, CHUNK_SIZE) / GRID_SIZE);
            if (cell_x < 0 || cell_x > CHUNK_CELLS - 1)
            {
                return 0.5f;
            }

            const int cell_y = static_cast<int>(fmodf(offset_y, CHUNK_SIZE) / GRID_SIZE);
            if (cell_y < 0 || cell_y > CHUNK_CELLS - 1)
            {
                return 0.5f;
            }

            return static_cast<float>((*grid[chunk_x][chunk_y])[cell_x][cell_y]) / MAX_CELL_VALUE;
        }

        void set_occupancy(const float x, const float y, const float occupancy)
        {
            const float offset_x = x + X_OFFSET;
            const float offset_y = y + Y_OFFSET;

            const int chunk_x = static_cast<int>(offset_x / CHUNK_SIZE);
            if (chunk_x < 0 || chunk_x > MAP_CHUNKS_X - 1)
            {
                return;
            }
            
            const int chunk_y = static_cast<int>(offset_y / CHUNK_SIZE);
            if (chunk_y < 0 || chunk_y > MAP_CHUNKS_Y - 1)
            {
                return;
            }

            if (!grid[chunk_x][chunk_y])
            {
                grid[chunk_x][chunk_y] = std::make_shared<chunk_t>();
                for (uint i = 0; i < CHUNK_CELLS; i++)
                {
                    (*grid[chunk_x][chunk_y])[i].fill(0.5f);
                }
            }

            const int cell_x = static_cast<int>(fmodf(offset_x, CHUNK_SIZE) / GRID_SIZE);
            if (cell_x < 0 || cell_x > CHUNK_CELLS - 1)
            {
                return;
            }

            const int cell_y = static_cast<int>(fmodf(offset_y, CHUNK_SIZE) / GRID_SIZE);
            if (cell_y < 0 || cell_y > CHUNK_CELLS - 1)
            {
                return;
            }

            (*grid[chunk_x][chunk_y])[cell_x][cell_y] = static_cast<uint>(occupancy * MAX_CELL_VALUE);
        }
    };
}