#include "lidarfeed.h"

#include <QDebug>
#include <QHostAddress>
#include <QNetworkDatagram>
#include <QtEndian>

LidarFeed::LidarFeed(QObject *parent)
    : QObject(parent)
{
    m_staleTimer.setInterval(500);
    connect(&m_staleTimer, &QTimer::timeout, this, &LidarFeed::checkStale);
    m_staleTimer.start();
    connect(&m_socket, &QUdpSocket::readyRead, this, &LidarFeed::readPending);
}

bool LidarFeed::listen(quint16 port)
{
    if (!m_socket.bind(QHostAddress::LocalHost, port, QUdpSocket::ShareAddress)) {
        qWarning() << "Lidar feed: cannot bind UDP port" << port << m_socket.errorString();
        return false;
    }
    return true;
}

void LidarFeed::readPending()
{
    while (m_socket.hasPendingDatagrams()) {
        const QByteArray data = m_socket.receiveDatagram().data();
        const int count = data.size() / 4;
        QList<QPointF> scan;
        scan.reserve(count);
        const uchar *raw = reinterpret_cast<const uchar *>(data.constData());
        for (int i = 0; i < count; ++i) {
            const qint16 x = qFromLittleEndian<qint16>(raw + i * 4);
            const qint16 y = qFromLittleEndian<qint16>(raw + i * 4 + 2);
            scan.append(QPointF(x / 100.0, y / 100.0));
        }
        setScan(scan);
    }
}

void LidarFeed::setScan(const QList<QPointF> &scan)
{
    m_points = scan;
    m_sinceScan.restart();
    emit pointsChanged();
    if (!m_streaming) {
        m_streaming = true;
        emit streamingChanged();
    }
}

void LidarFeed::checkStale()
{
    if (m_streaming && (!m_sinceScan.isValid() || m_sinceScan.elapsed() > 1500)) {
        m_streaming = false;
        emit streamingChanged();
    }
}
