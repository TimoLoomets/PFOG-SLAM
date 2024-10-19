#include <gtest/gtest.h>
#include "geometry.hpp"

// Test Fixture for Geometry
class GeometryTest : public ::testing::Test {
protected:
    void SetUp() override {
    }

    void TearDown() override {
    }
};

TEST_F(GeometryTest, ZShape) {
    float x0 = 0.5f;
    float y0 = 0.5f;

    float x1 = 2.5f;
    float y1 = 1.5f;
    
    auto cells = geometry::DDALine(x0, y0, x1, y1);
    for(const auto& cell : cells)
    {
        std::cout << cell.x << " " << cell.y <<",";
    }
}

TEST_F(GeometryTest, VerticalLine) {
    float x0 = 0.5f;
    float y0 = 0.5f;

    float x1 = 0.5f;
    float y1 = 5.5f;
    
    auto cells = geometry::DDALine(x0, y0, x1, y1);
    for(const auto& cell : cells)
    {
        std::cout << cell.x << " " << cell.y <<",";
    }
}

TEST_F(GeometryTest, HorizontalLine) {
    float x0 = 0.5f;
    float y0 = 0.5f;

    float x1 = 5.5f;
    float y1 = 0.5f;
    
    auto cells = geometry::DDALine(x0, y0, x1, y1);
    for(const auto& cell : cells)
    {
        std::cout << cell.x << " " << cell.y <<",";
    }
}
