#pragma once
#include "vec2d.hpp"

namespace geometry
{
    class Ray
    {
        public:
        vec2d<float> source;
        vec2d<float> direction;
        float length;
    };
}