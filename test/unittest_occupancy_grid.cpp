#include <gtest/gtest.h>
#include "occupancy_grid.hpp"

// Test Fixture for OccupancyGrid
class OccupancyGridTest : public ::testing::Test {
protected:
    SLAM::OccupancyGrid<15> grid; // Test instance of the grid

    void SetUp() override {
    }

    void TearDown() override {
    }
};

// Test default occupancy value of an uninitialized grid cell
TEST_F(OccupancyGridTest, DefaultOccupancy) {
    float x = 5.0;
    float y = 5.0;
    
    // Assuming that default occupancy is 0.0 for unoccupied cells
    EXPECT_NEAR(grid.get_occupancy(x, y), 0.5f, 0.01f);
}

// Test setting and getting occupancy for a specific grid cell
TEST_F(OccupancyGridTest, SetAndGetOccupancy) {
    float x = 2.0;
    float y = 3.0;
    float occupancy_value = 0.7;
    
    // Set the occupancy value
    grid.set_occupancy(x, y, occupancy_value);
    
    // Get the occupancy value and check if it matches
    EXPECT_NEAR(grid.get_occupancy(x, y), occupancy_value, 0.01f);
}

// Test that setting occupancy does not affect other cells
TEST_F(OccupancyGridTest, OccupancyIndependence) {
    float x1 = 1.0;
    float y1 = 1.0;
    float x2 = 4.0;
    float y2 = 4.0;
    
    float occupancy_value_1 = 0.6f;
    float occupancy_value_2 = 0.8f;
    
    // Set occupancy for the first cell
    grid.set_occupancy(x1, y1, occupancy_value_1);
    
    // Ensure setting one cell does not affect another
    EXPECT_NEAR(grid.get_occupancy(x2, y2), 0.5f, 0.01f);
    
    // Set occupancy for the second cell
    grid.set_occupancy(x2, y2, occupancy_value_2);
    
    // Verify both cells have their respective values
    EXPECT_NEAR(grid.get_occupancy(x1, y1), occupancy_value_1, 0.01f);
    EXPECT_NEAR(grid.get_occupancy(x2, y2), occupancy_value_2, 0.01f);
}

