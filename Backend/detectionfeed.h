#pragma once

#include <QObject>
#include <QUdpSocket>

class DetectionModel;

// Receives one JSON datagram per tick from tools/carla_vesc_bridge.py:
// {"objects":[{"id":1,"class":"person","x":7.0,"y":-2.0,"d":7.3,"src":"fused","vx":0.0,"vy":0.0}]}
// x is metres forward of the lidar, y metres to the right.
class DetectionFeed : public QObject
{
    Q_OBJECT

public:
    DetectionFeed(DetectionModel *model, QObject *parent = nullptr);

    bool listen(quint16 port);

private:
    void readPending();

    QUdpSocket m_socket;
    DetectionModel *m_model;
};
