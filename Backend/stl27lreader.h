#pragma once

#include <QByteArray>
#include <QList>
#include <QObject>
#include <QPointF>
#include <QSerialPort>
#include <QString>
#include <QTimer>

// Reads an LDROBOT STL-27L over UART and emits one complete revolution at a time.
//
// Wire format (little-endian, 47 bytes per packet, ~450 packets/s at 921600 8N1):
//   0x54, 0x2C, speed(u16, deg/s), start_angle(u16, 0.01 deg),
//   12 x { distance(u16, mm), intensity(u8) },
//   end_angle(u16, 0.01 deg), timestamp(u16, ms), crc8
// Points are 21600/s, about 2160 per revolution at 10 Hz. Angles run clockwise seen from above,
// 0 deg is the sensor's front.
//
// Output points are metres in the kart frame relative to the lidar: x forward, y right.
class Stl27lReader : public QObject
{
    Q_OBJECT
    Q_PROPERTY(qint64 goodPackets READ goodPackets NOTIFY statsChanged)
    Q_PROPERTY(qint64 badPackets READ badPackets NOTIFY statsChanged)

public:
    explicit Stl27lReader(QObject *parent = nullptr);

    bool open(const QString &device, qint32 baud = 921600);

    // Mount yaw: degrees the sensor's 0 deg direction is rotated clockwise from the kart's forward.
    void setMountYawDeg(double deg) { m_yawDeg = deg; }
    // Set when the sensor is mounted upside down (angles then run counter-clockwise).
    void setMirrored(bool mirrored) { m_mirrored = mirrored; }
    void setRangeLimitsM(double minM, double maxM) { m_minM = minM; m_maxM = maxM; }
    // Drop returns whose sensor angle is in [fromDeg, toDeg] (e.g. the kart body or driver).
    void addMaskDeg(double fromDeg, double toDeg) { m_masks.append({fromDeg, toDeg}); }

    qint64 goodPackets() const { return m_good; }
    qint64 badPackets() const { return m_bad; }

    // Exposed for tests.
    static quint8 crc8(const quint8 *data, int len);
    void feed(const QByteArray &bytes);

signals:
    void scanReady(const QList<QPointF> &points);
    void statsChanged();

private:
    void readSerial();
    void handleError();
    void reopen();
    void decodePacket(const quint8 *p);
    bool masked(double sensorDeg) const;

    QSerialPort m_port;
    QTimer m_retryTimer;
    QString m_device;
    qint32 m_baud = 921600;
    QByteArray m_buffer;
    QList<QPointF> m_scan;
    double m_lastDeg = -1;
    double m_yawDeg = 0;
    bool m_mirrored = false;
    double m_minM = 0.05;
    double m_maxM = 25.0;
    struct Mask { double from; double to; };
    QList<Mask> m_masks;
    qint64 m_good = 0;
    qint64 m_bad = 0;
};
