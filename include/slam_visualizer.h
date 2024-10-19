// slam_visualizer.h

#ifndef SLAM_VISUALIZER_H
#define SLAM_VISUALIZER_H

#include <QObject>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsLineItem>
#include "slam.h"

class SLAMVisualizer : public QObject {
    Q_OBJECT

public:
    SLAMVisualizer(SLAM& slam, QGraphicsScene* scene);
    void update();

private:
    SLAM& slam;
    QGraphicsScene* scene;
    QGraphicsEllipseItem* robotItem;

    void drawSensorRay(Pose pose, double angle, double distance);
};

#endif // SLAM_VISUALIZER_H
