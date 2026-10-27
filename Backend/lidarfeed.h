#pragma once

#include <QElapsedTimer>
#include <QList>
#include <QObject>
#include <QPointF>
#include <QTimer>
#include <QUdpSocket>

// Receives low-density lidar scans from tools/carla_vesc_bridge.py.
// Each UDP datagram is one scan: little-endian int16 pairs (x forward, y right) in centimetres.
class LidarFeed : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QList<QPointF> points READ points NOTIFY pointsChanged)
    Q_PROPERTY(bool streaming READ streaming NOTIFY streamingChanged)

public:
    explicit LidarFeed(QObject *parent = nullptr);

    bool listen(quint16 port);
    void setScan(const QList<QPointF> &scan);
    QList<QPointF> points() const { return m_points; }
    bool streaming() const { return m_streaming; }

signals:
    void pointsChanged();
    void streamingChanged();

private:
    void readPending();
    void checkStale();

    QUdpSocket m_socket;
    QTimer m_staleTimer;
    QElapsedTimer m_sinceScan;
    QList<QPointF> m_points;
    bool m_streaming = false;
};
