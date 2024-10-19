#pragma once
#include <math.h>

namespace
{
    template<typename T>
    class vec2d
    {
        public:
        T x;
        T y;

        T length() const
        {
            return sqrt(x * x + y * y);
        }

        vec2d<T> normalize() const
        {
            T len = length();
            if (len > 0.0f)
                return {x / len, y / len};
            return {0, 0};
        }
    };
}