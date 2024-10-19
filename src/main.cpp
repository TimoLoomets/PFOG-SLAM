#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QTimer>
#include "slam.h"
#include "slam_visualizer.h"  // Include the new SLAMVisualizer header

// Main function for the Qt GUI application
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Create SLAM system
    SLAM slam;
    Pose initial_pose = {MAP_WIDTH / 2, MAP_HEIGHT / 2, 0.0}; // Set robot at map center
    slam.motion_update(1.0, 0.0, 0.1);  // Simulate motion
    std::vector<double> measurements = slam.grid_map.sensor_measurements(initial_pose);
    slam.sensor_update(measurements);   // Update SLAM with simulated sensor data

    // Create a QGraphicsScene to visualize the map and robot
    QGraphicsScene scene;
    scene.setSceneRect(0, 0, MAP_WIDTH / GRID_RESOLUTION * 10, MAP_HEIGHT / GRID_RESOLUTION * 10);
    QGraphicsView view(&scene);
    view.setWindowTitle("SLAM Visualization");
    view.show();

    // Set up the visualizer with a timer to update regularly
    SLAMVisualizer visualizer(slam, &scene);
    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() { visualizer.update(); });
    timer.start(100);  // Update visualization every 100 ms

    // Execute the Qt application
    return app.exec();
}
