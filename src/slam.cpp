#include "slam.h"
#include <cmath>
#include <algorithm>
#include <iostream>

// Constructor
SLAM::SLAM() {
    // Initialize the grid map with a certain size
    grid_map = OccupancyGridMap(MAP_WIDTH, MAP_HEIGHT, GRID_RESOLUTION);
    estimated_pose = {MAP_WIDTH / 2, MAP_HEIGHT / 2, 0.0};  // Initial pose at the center of the map
}

// Motion update function
void SLAM::motion_update(double delta_x, double delta_y, double delta_theta) {
    // Update the pose of the robot based on motion
    estimated_pose.x += delta_x;
    estimated_pose.y += delta_y;
    estimated_pose.theta += delta_theta;

    // Normalize theta to be within [-PI, PI]
    if (estimated_pose.theta > PI) estimated_pose.theta -= 2 * PI;
    if (estimated_pose.theta < -PI) estimated_pose.theta += 2 * PI;
}

// Sensor update function
void SLAM::sensor_update(const std::vector<double>& measurements) {
    // Update the occupancy grid based on sensor measurements
    for (size_t i = 0; i < measurements.size(); ++i) {
        double angle = estimated_pose.theta + (i - 2) * PI / 4;  // Map sensor index to angle
        double distance = measurements[i];

        // Compute endpoint of the ray
        double end_x = estimated_pose.x + cos(angle) * distance;
        double end_y = estimated_pose.y + sin(angle) * distance;

        // Mark cells in the grid as occupied or free
        grid_map.update_occupancy(estimated_pose.x, estimated_pose.y, end_x, end_y);
    }
}

// Function to get the estimated pose
Pose SLAM::get_estimated_pose() const {
    return estimated_pose;
}

// Occupancy grid implementation
OccupancyGridMap::OccupancyGridMap(double width, double height, double resolution)
    : width(width), height(height), resolution(resolution) {
    grid_width = static_cast<int>(width / resolution);
    grid_height = static_cast<int>(height / resolution);
    grid.resize(grid_width * grid_height, 0.5);  // Initialize with unknown occupancy
}

// Function to update occupancy in the grid
void OccupancyGridMap::update_occupancy(double start_x, double start_y, double end_x, double end_y) {
    // Convert coordinates to grid indices
    int start_i = static_cast<int>(start_x / resolution);
    int start_j = static_cast<int>(start_y / resolution);
    int end_i = static_cast<int>(end_x / resolution);
    int end_j = static_cast<int>(end_y / resolution);

    // Mark the start cell as occupied
    if (is_within_bounds(start_i, start_j)) {
        grid[start_j * grid_width + start_i] = 1.0;  // Occupied
    }

    // Mark the end cell as free
    if (is_within_bounds(end_i, end_j)) {
        grid[end_j * grid_width + end_i] = 0.0;  // Free
    }

    // Implement ray tracing between start and end to mark other cells
    // Simple Bresenham's line algorithm for grid marking
    int dx = std::abs(end_i - start_i);
    int dy = std::abs(end_j - start_j);
    int sx = (start_i < end_i) ? 1 : -1;
    int sy = (start_j < end_j) ? 1 : -1;
    int err = dx - dy;

    while (true) {
        if (is_within_bounds(start_i, start_j)) {
            grid[start_j * grid_width + start_i] = 0.0;  // Mark as free space
        }

        if (start_i == end_i && start_j == end_j) break;
        int err2 = err * 2;

        if (err2 > -dy) {
            err -= dy;
            start_i += sx;
        }
        if (err2 < dx) {
            err += dx;
            start_j += sy;
        }
    }
}

// Function to check if a grid cell is within bounds
bool OccupancyGridMap::is_within_bounds(int i, int j) const {
    return (i >= 0 && i < grid_width && j >= 0 && j < grid_height);
}

// Function to get occupancy value at a specific position
double OccupancyGridMap::get(double x, double y) const {
    int i = static_cast<int>(x / resolution);
    int j = static_cast<int>(y / resolution);
    if (is_within_bounds(i, j)) {
        return grid[j * grid_width + i];
    }
    return -1;  // Out of bounds
}

// Function to simulate sensor measurements (to be called during initialization or updates)
std::vector<double> OccupancyGridMap::sensor_measurements(const Pose& pose) {
    // Simulated measurements from 5 sensors
    std::vector<double> measurements(5);
    for (size_t i = 0; i < measurements.size(); ++i) {
        double angle = pose.theta + (i - 2) * PI / 4;  // Map sensor index to angle
        measurements[i] = 5.0;  // Replace this with actual distance calculations
    }
    return measurements;
}
