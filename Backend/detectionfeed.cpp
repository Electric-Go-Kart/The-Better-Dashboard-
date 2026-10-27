#include "detectionfeed.h"

#include "detectionmodel.h"

#include <QDebug>
#include <QHostAddress>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QNetworkDatagram>
#include <QtMath>

DetectionFeed::DetectionFeed(DetectionModel *model, QObject *parent)
    : QObject(parent), m_model(model)
{
    connect(&m_socket, &QUdpSocket::readyRead, this, &DetectionFeed::readPending);
}

bool DetectionFeed::listen(quint16 port)
{
    if (!m_socket.bind(QHostAddress::LocalHost, port, QUdpSocket::ShareAddress)) {
        qWarning() << "Detection feed: cannot bind UDP port" << port << m_socket.errorString();
        return false;
    }
    return true;
}

void DetectionFeed::readPending()
{
    QByteArray latest;
    while (m_socket.hasPendingDatagrams())
        latest = m_socket.receiveDatagram().data();
    if (latest.isEmpty() || m_model->demo())
        return;

    const QJsonDocument doc = QJsonDocument::fromJson(latest);
    if (!doc.isObject())
        return;

    QList<Detection> list;
    const QJsonArray objects = doc.object().value("objects").toArray();
    list.reserve(objects.size());
    for (const QJsonValue &v : objects) {
        const QJsonObject o = v.toObject();
        Detection d;
        d.id = o.value("id").toInt();
        d.classId = o.value("class").toString("unknown");
        d.x = o.value("x").toDouble();
        d.y = o.value("y").toDouble();
        d.distance = o.contains("d") ? o.value("d").toDouble() : qSqrt(d.x * d.x + d.y * d.y);
        d.source = o.value("src").toString("lidar");
        d.confidence = o.value("conf").toDouble(1.0);
        d.vx = o.value("vx").toDouble();
        d.vy = o.value("vy").toDouble();
        list.append(d);
    }
    m_model->setDetections(list);
}
