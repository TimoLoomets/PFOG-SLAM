cmake_minimum_required(VERSION 3.5)
project(SLAM)

set(CMAKE_CXX_STANDARD 14)

# Enable automoc for handling Q_OBJECT automatically
set(CMAKE_AUTOMOC ON)

# Find the Qt5 package
find_package(Qt5 COMPONENTS Widgets REQUIRED)

# Include the directories with the SLAM classes
include_directories(${CMAKE_SOURCE_DIR}/include)

enable_testing()
find_package(GTest)

if(GTest_FOUND)
    message(STATUS "Found Google Test. Using installed version.")
    include_directories(${GTEST_INCLUDE_DIRS})
else()
    message(STATUS "Google Test not found. Fetching Google Test from source.")

    # Option 2: Fetch GoogleTest using FetchContent
    include(FetchContent)
    FetchContent_Declare(
      googletest
      URL https://github.com/google/googletest/archive/release-1.15.2.zip
    )
    # For Windows: Prevent overriding the parent project's compiler/linker settings
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    # Include GTest headers
    include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
endif()

find_package(benchmark REQUIRED)




add_executable(test_dynamic_occupancy_grid
    test/unittest_dynamic_occupancy_grid.cpp
)
target_link_libraries(test_dynamic_occupancy_grid ${GTEST_LIBRARIES} GTest::GTest GTest::Main pthread)
add_test(NAME DynamicOccupancyGridTest COMMAND test_dynamic_occupancy_grid)

add_executable(benchmark_dynamic_occupancy_grid
    test/performancetest_dynamic_occupancy_grid.cpp
)
target_link_libraries(benchmark_dynamic_occupancy_grid benchmark::benchmark)


add_executable(test_occupancy_grid
    test/unittest_occupancy_grid.cpp
)
target_link_libraries(test_occupancy_grid ${GTEST_LIBRARIES} GTest::GTest GTest::Main pthread)
add_test(NAME OccupancyGridTest COMMAND test_occupancy_grid)

add_executable(benchmark_occupancy_grid
    test/performancetest_occupancy_grid.cpp
)
target_link_libraries(benchmark_occupancy_grid benchmark::benchmark)


add_executable(test_geometry
    test/unittest_geometry.cpp
)
target_link_libraries(test_geometry ${GTEST_LIBRARIES} GTest::GTest GTest::Main pthread)
# Add the source files
#set(SOURCES
#    src/slam.cpp
#    src/slam_gui.cpp
#    src/slam_visualizer.cpp
#    src/main.cpp
#)

#add_executable(SLAM_Visualization ${SOURCES})

# Link against the Qt5 Widgets library
#target_link_libraries(SLAM_Visualization Qt5::Widgets)
