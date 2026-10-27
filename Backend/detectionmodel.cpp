#include "detectionmodel.h"

#include <QtMath>

DetectionModel::DetectionModel(QObject *parent)
    : QAbstractListModel(parent)
{
    m_demoTimer.setInterval(66);
    connect(&m_demoTimer, &QTimer::timeout, this, &DetectionModel::tickDemo);
}

int DetectionModel::rowCount(const QModelIndex &parent) const
{
    return parent.isValid() ? 0 : m_items.size();
}

QVariant DetectionModel::data(const QModelIndex &index, int role) const
{
    if (!index.isValid() || index.row() >= m_items.size())
        return {};
    const Detection &d = m_items.at(index.row());
    switch (role) {
    case IdRole: return d.id;
    case ClassRole: return d.classId;
    case XRole: return d.x;
    case YRole: return d.y;
    case DistanceRole: return d.distance;
    case SourceRole: return d.source;
    case ConfidenceRole: return d.confidence;
    case VxRole: return d.vx;
    case VyRole: return d.vy;
    }
    return {};
}

QHash<int, QByteArray> DetectionModel::roleNames() const
{
    return {
        {IdRole, "objectId"},
        {ClassRole, "objectClass"},
        {XRole, "posX"},
        {YRole, "posY"},
        {DistanceRole, "distance"},
        {SourceRole, "source"},
        {ConfidenceRole, "confidence"},
        {VxRole, "velX"},
        {VyRole, "velY"},
    };
}

void DetectionModel::setDemo(bool on)
{
    if (m_demo == on)
        return;
    m_demo = on;
    if (on)
        m_demoTimer.start();
    else
        m_demoTimer.stop();
    emit demoChanged();
}

void DetectionModel::setDetections(const QList<Detection> &detections)
{
    // Same object set: update values in place so QML delegates (and their animations) persist.
    bool sameIds = detections.size() == m_items.size();
    for (int i = 0; sameIds && i < detections.size(); ++i)
        sameIds = detections.at(i).id == m_items.at(i).id;

    if (sameIds) {
        m_items = detections;
        if (!m_items.isEmpty())
            emit dataChanged(index(0), index(m_items.size() - 1));
    } else {
        beginResetModel();
        m_items = detections;
        endResetModel();
    }

    double closest = -1;
    for (const Detection &d : m_items)
        if (closest < 0 || d.distance < closest)
            closest = d.distance;
    if (!qFuzzyCompare(closest + 1, m_closest + 1)) {
        m_closest = closest;
        emit closestChanged();
    }
}

// Synthetic objects for developing the UI without CARLA or a detector.
void DetectionModel::tickDemo()
{
    m_demoT += 0.066;
    const double t = m_demoT;
    auto make = [](int id, const QString &cls, double x, double y, const QString &src) {
        Detection d;
        d.id = id;
        d.classId = cls;
        d.x = x;
        d.y = y;
        d.distance = qSqrt(x * x + y * y);
        d.source = src;
        return d;
    };
    QList<Detection> list;
    list << make(1, "person", 7.0 + 2.0 * qSin(t * 0.5), -2.0 + 1.5 * qSin(t * 0.8), "fused");
    list.last().vx = qCos(t * 0.5);
    list.last().vy = 1.2 * qCos(t * 0.8);
    list << make(2, "vehicle", 15.0, 4.0 + 1.0 * qSin(t * 0.3), "fused");
    list << make(3, "cone", 4.0, 1.5, "fused");
    list << make(8, "pole/barrier", 9.0, 6.0, "fused");
    list << make(4, "bike", 11.0 + 3.0 * qCos(t * 0.4), -5.0, "camera");
    list << make(5, "unknown", -6.0 + 1.5 * qSin(t * 0.6), 3.0, "lidar");
    list << make(6, "unknown", 1.0, -8.0, "lidar");
    list << make(7, "unknown", -14.0, -4.0 + 2.0 * qCos(t * 0.5), "lidar");
    setDetections(list);
}
