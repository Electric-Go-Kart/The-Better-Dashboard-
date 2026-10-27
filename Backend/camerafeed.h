#pragma once

#include <QElapsedTimer>
#include <QImage>
#include <QMutex>
#include <QObject>
#include <QQuickImageProvider>
#include <QTimer>
#include <QUdpSocket>
#include <memory>

// Latest camera frame, shared between the UDP receiver and the QML image provider.
class CameraFrameStore
{
public:
    void set(const QImage &image)
    {
        QMutexLocker lock(&m_mutex);
        m_image = image;
    }

    QImage get() const
    {
        QMutexLocker lock(&m_mutex);
        return m_image;
    }

private:
    mutable QMutex m_mutex;
    QImage m_image;
};

// Receives one JPEG per UDP datagram (sent by tools/carla_vesc_bridge.py).
class CameraFeed : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int frameCount READ frameCount NOTIFY frameCountChanged)
    Q_PROPERTY(bool streaming READ streaming NOTIFY streamingChanged)

public:
    explicit CameraFeed(QObject *parent = nullptr);

    bool listen(quint16 port);
    std::shared_ptr<CameraFrameStore> store() const { return m_store; }
    int frameCount() const { return m_frameCount; }
    bool streaming() const { return m_streaming; }

signals:
    void frameCountChanged();
    void streamingChanged();

private:
    void readPending();
    void checkStale();

    QUdpSocket m_socket;
    QTimer m_staleTimer;
    QElapsedTimer m_sinceFrame;
    std::shared_ptr<CameraFrameStore> m_store;
    int m_frameCount = 0;
    bool m_streaming = false;
};

class CameraImageProvider : public QQuickImageProvider
{
public:
    explicit CameraImageProvider(std::shared_ptr<CameraFrameStore> store)
        : QQuickImageProvider(QQuickImageProvider::Image), m_store(std::move(store)) {}

    QImage requestImage(const QString &, QSize *size, const QSize &) override
    {
        const QImage image = m_store->get();
        if (size)
            *size = image.size();
        return image;
    }

private:
    std::shared_ptr<CameraFrameStore> m_store;
};
