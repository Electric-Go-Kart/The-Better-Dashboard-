#include "camerafeed.h"

#include <QDebug>
#include <QHostAddress>
#include <QNetworkDatagram>

CameraFeed::CameraFeed(QObject *parent)
    : QObject(parent), m_store(std::make_shared<CameraFrameStore>())
{
    m_staleTimer.setInterval(500);
    connect(&m_staleTimer, &QTimer::timeout, this, &CameraFeed::checkStale);
    m_staleTimer.start();
    connect(&m_socket, &QUdpSocket::readyRead, this, &CameraFeed::readPending);
}

bool CameraFeed::listen(quint16 port)
{
    if (!m_socket.bind(QHostAddress::LocalHost, port, QUdpSocket::ShareAddress)) {
        qWarning() << "Camera feed: cannot bind UDP port" << port << m_socket.errorString();
        return false;
    }
    return true;
}

void CameraFeed::readPending()
{
    while (m_socket.hasPendingDatagrams()) {
        const QByteArray data = m_socket.receiveDatagram().data();
        const QImage image = QImage::fromData(data, "JPEG");
        if (image.isNull())
            continue;
        m_store->set(image);
        m_sinceFrame.restart();
        ++m_frameCount;
        emit frameCountChanged();
        if (!m_streaming) {
            m_streaming = true;
            emit streamingChanged();
        }
    }
}

void CameraFeed::checkStale()
{
    if (m_streaming && (!m_sinceFrame.isValid() || m_sinceFrame.elapsed() > 1500)) {
        m_streaming = false;
        emit streamingChanged();
    }
}
