#pragma once

namespace SLAM
{
    class occupancy_grid_template
    {
        public:
        virtual float get_occupancy(const float x, const float y) const = 0;
        virtual void set_occupancy(const float x, const float y, const float occupancy) = 0;
    };
}