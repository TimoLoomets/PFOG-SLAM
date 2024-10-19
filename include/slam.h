#ifndef SLAM_H
#define SLAM_H

#include <vector>
#include <random>
#include <cmath>

const double MAP_WIDTH = 10.0;  // Map width in meters
const double MAP_HEIGHT = 10.0; // Map height in meters
const int NUM_PARTICLES = 100;
const double MOVE_NOISE = 0.1;
const double SENSOR_NOISE = 0.5;
const double GRID_RESOLUTION = 0.1; // Each grid cell represents 0.1 meters
const double MAX_SENSOR_RANGE = 10.0; // Maximum range of the sensor in meters
const double PI = 3.14159265359;

// Calculate grid dimensions based on map size and grid resolution
const int GRID_WIDTH = static_cast<int>(MAP_WIDTH / GRID_RESOLUTION);
const int GRID_HEIGHT = static_cast<int>(MAP_HEIGHT / GRID_RESOLUTION);

// Structure to represent the robot pose (x, y, theta)
struct Pose {
    double x, y, theta;
};

// Occupancy Grid Map class
class OccupancyGridMap {
public:
    std::vector<std::vector<double>> occupancy_grid;

    OccupancyGridMap();

    void update(double x, double y, double probability);
    double get(double x, double y);
    double ray_casting(Pose pose, double angle);
    std::vector<double> sensor_measurements(Pose pose);
};

// Particle class
struct Particle {
    Pose pose;
    double weight;
};

// Particle Filter class
class ParticleFilter {
public:
    std::vector<Particle> particles;
    std::default_random_engine generator;

    ParticleFilter();
    void motion_update(double delta_x, double delta_y, double delta_theta);
    void sensor_update(OccupancyGridMap& map, const std::vector<double>& measurements);
    void resample();
    Pose estimate_position();

private:
    double gaussian(double mu, double sigma, double x);
    void normalize_weights();
};

// SLAM system class
class SLAM {
public:
    OccupancyGridMap grid_map;
    ParticleFilter particle_filter;

    SLAM();

    void motion_update(double delta_x, double delta_y, double delta_theta);
    void sensor_update(const std::vector<double>& measurements);
    void update_map(double x, double y, double probability);
    Pose get_estimated_pose();
};

#endif // SLAM_H
