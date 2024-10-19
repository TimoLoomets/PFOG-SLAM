#pragma once
#include "vec2d.hpp"
#include <math.h>
#include <limits>


namespace geometry
{
    std::vector<vec2d<int>> DDALine(float x0, float y0, float x1, float y1) 
    { 
        const vec2d<float> vec = {x1 - x0, y1 - y0};
		const vec2d<float> ray_direction = vec.normalize();

		const vec2d<float> unit_step = {
            fabsf(ray_direction.x) > 0.0f ? sqrt(1 + (ray_direction.y / ray_direction.x) * (ray_direction.y / ray_direction.x)) : std::numeric_limits<float>::infinity()
            , fabsf(ray_direction.y) > 0.0f ? sqrt(1 + (ray_direction.x / ray_direction.y) * (ray_direction.x / ray_direction.y)) : std::numeric_limits<float>::infinity()
        };

		vec2d<int> cell = {static_cast<int>(x0), static_cast<int>(y0)};
		vec2d<float> ray_length_1D;
		vec2d<int> step;

		// Establish Starting Conditions
		if (ray_direction.x < 0)
		{
			step.x = -1;
			ray_length_1D.x = (x0 - float(cell.x)) * unit_step.x;
		}
		else
		{
			step.x = 1;
			ray_length_1D.x = (float(cell.x + 1) - x0) * unit_step.x;
		}

		if (ray_direction.y < 0)
		{
			step.y = -1;
			ray_length_1D.y = (y0 - float(cell.y)) * unit_step.y;
		}
		else
		{
			step.y = 1;
			ray_length_1D.y = (float(cell.y + 1) - y0) * unit_step.y;
		}

        std::vector<vec2d<int>> cells;
        cells.push_back(cell);
		while (cell.x != static_cast<int>(x1) || cell.y != static_cast<int>(y1))
		{
			// Walk along shortest path
			if (ray_length_1D.x < ray_length_1D.y)
			{
				cell.x += step.x;
				ray_length_1D.x += unit_step.x;
			}
			else
			{
				cell.y += step.y;
				ray_length_1D.y += unit_step.y;
			}
            cells.push_back(cell);
		}

        return cells;
    }
}