#ifndef DRIVESTATEMANAGER_H
#define DRIVESTATEMANAGER_H

#include <QObject>
#include <QQmlEngine>

class DriveStateManager : public QObject {
    Q_OBJECT
    QML_ELEMENT

public:
    enum class DriveState {
        DriveForward,
        DriveReverse,
        Park,
        Fault
    };
    Q_ENUM(DriveState)

    explicit DriveStateManager(QObject *parent = nullptr);

    DriveState currentState() const;
    bool isParkEngaged() const;
    bool isReverseEnabled() const;
    bool isFaulted() const;
    QString stateLabel() const;
    QString lastRejectionReason() const;

public slots:
    bool requestPark(bool enabled);
    bool requestReverse(bool enabled);
    void setFaulted(bool faulted);
    void updateSpeedRpm(int leftRpm, int rightRpm);

signals:
    void stateChanged(DriveStateManager::DriveState state);
    void stateLabelChanged(const QString &label);
    void parkChanged(bool engaged);
    void reverseChanged(bool reverseEnabled);
    void faultChanged(bool faulted);
    void transitionRejected(const QString &reason);

private:
    bool canExitPark() const;
    bool canEnterReverse() const;
    void setState(DriveStateManager::DriveState nextState);

    DriveState state = DriveState::Park;
    int lastLeftRpm = 0;
    int lastRightRpm = 0;
    int safeRpmThreshold = 80;
    QString rejectionReason;
};

#endif
