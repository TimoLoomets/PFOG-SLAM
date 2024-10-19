#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QTimer>
#include <QPen>
#include <QBrush>
#include "slam.h"

// SLAM Visualizer class for rendering
class SLAMVisualizer : public QObject {
    Q_OBJECT

public:
    SLAMVisualizer(SLAM& slam, QGraphicsScene* scene)
        : slam(slam), scene(scene) {
        robotItem = scene->addEllipse(0, 0, 10, 10, QPen(Qt::blue), QBrush(Qt::blue)); // Robot as a circle
    }

    void update() {
        scene->clear();

        // Draw the occupancy grid map
        for (int i = 0; i < GRID_WIDTH; ++i) {
            for (int j = 0; j < GRID_HEIGHT; ++j) {
                double occupancy = slam.grid_map.get(i * GRID_RESOLUTION, j * GRID_RESOLUTION);
                QColor color = occupancy > 0.5 ? Qt::black : Qt::white;
                scene->addRect(i * 10, j * 10, 10, 10, QPen(Qt::gray), QBrush(color));
            }
        }

        // Get and visualize robot's pose
        Pose robotPose = slam.get_estimated_pose();
        double robotX = robotPose.x / GRID_RESOLUTION * 10;
        double robotY = robotPose.y / GRID_RESOLUTION * 10;

        robotItem->setRect(robotX, robotY, 10, 10);
        robotItem->setRotation(robotPose.theta * 180 / PI);  // Convert radians to degrees for Qt

        // Visualize sensor rays
        std::vector<double> sensorMeasurements = slam.grid_map.sensor_measurements(robotPose);
        drawSensorRay(robotPose, -PI / 2, sensorMeasurements[0]);
        drawSensorRay(robotPose, -PI / 4, sensorMeasurements[1]);
        drawSensorRay(robotPose, 0, sensorMeasurements[2]);
        drawSensorRay(robotPose, PI / 4, sensorMeasurements[3]);
        drawSensorRay(robotPose, PI / 2, sensorMeasurements[4]);
    }

private:
    SLAM& slam;
    QGraphicsScene* scene;
    QGraphicsEllipseItem* robotItem;

    // Draw a ray to visualize sensor data
    void drawSensorRay(Pose pose, double angle, double distance) {
        double sensorX = pose.x + cos(pose.theta + angle) * distance;
        double sensorY = pose.y + sin(pose.theta + angle) * distance;

        scene->addLine(pose.x / GRID_RESOLUTION * 10,
                       pose.y / GRID_RESOLUTION * 10,
                       sensorX / GRID_RESOLUTION * 10,
                       sensorY / GRID_RESOLUTION * 10,
                       QPen(Qt::red));
    }
};

// Add this line at the end of the file
#include "slam_gui.moc"
