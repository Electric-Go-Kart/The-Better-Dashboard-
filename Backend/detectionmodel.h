#pragma once

#include <QAbstractListModel>
#include <QList>
#include <QString>
#include <QTimer>

struct Detection
{
    int id = 0;
    QString classId;   // person, vehicle, cone, bike, unknown
    double x = 0;      // m, forward of the kart
    double y = 0;      // m, right of the kart
    double distance = 0;
    QString source;    // fused, lidar, camera
    double confidence = 1.0;
    double vx = 0;     // m/s, forward
    double vy = 0;     // m/s, right
};

// Objects shown on the What The Kart Sees bird's-eye view.
class DetectionModel : public QAbstractListModel
{
    Q_OBJECT
    Q_PROPERTY(double closestDistance READ closestDistance NOTIFY closestChanged)
    Q_PROPERTY(bool demo READ demo WRITE setDemo NOTIFY demoChanged)

public:
    enum Roles {
        IdRole = Qt::UserRole + 1,
        ClassRole,
        XRole,
        YRole,
        DistanceRole,
        SourceRole,
        ConfidenceRole,
        VxRole,
        VyRole
    };

    explicit DetectionModel(QObject *parent = nullptr);

    int rowCount(const QModelIndex &parent = QModelIndex()) const override;
    QVariant data(const QModelIndex &index, int role) const override;
    QHash<int, QByteArray> roleNames() const override;

    double closestDistance() const { return m_closest; }
    bool demo() const { return m_demo; }
    void setDemo(bool on);

    void setDetections(const QList<Detection> &detections);

signals:
    void closestChanged();
    void demoChanged();

private:
    void tickDemo();

    QList<Detection> m_items;
    double m_closest = -1;
    bool m_demo = false;
    QTimer m_demoTimer;
    double m_demoT = 0;
};
