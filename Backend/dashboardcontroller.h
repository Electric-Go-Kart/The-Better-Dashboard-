#ifndef DASHBOARDCONTROLLER_H
#define DASHBOARDCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include "drivestatemanager.h"

class DashboardController : public QObject {
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(QString driveState READ driveState NOTIFY driveStateChanged)
    Q_PROPERTY(bool parkEngaged READ parkEngaged NOTIFY parkEngagedChanged)
    Q_PROPERTY(bool reverseEnabled READ reverseEnabled NOTIFY reverseEnabledChanged)
    Q_PROPERTY(bool lightsEnabled READ lightsEnabled NOTIFY lightsEnabledChanged)
    Q_PROPERTY(bool lockEnabled READ lockEnabled NOTIFY lockEnabledChanged)
    Q_PROPERTY(bool canOnline READ canOnline NOTIFY canOnlineChanged)
    Q_PROPERTY(QString canStatus READ canStatus NOTIFY canStatusChanged)
    Q_PROPERTY(QString lastControlError READ lastControlError NOTIFY lastControlErrorChanged)

public:
    explicit DashboardController(QObject *parent = nullptr);
    QString driveState() const;
    bool parkEngaged() const;
    bool reverseEnabled() const;
    bool lightsEnabled() const;
    bool lockEnabled() const;
    bool canOnline() const;
    QString canStatus() const;
    QString lastControlError() const;

signals:
    // Left Motor UI
    void leftRpmChanged(int rpm);
    void leftCurrentChanged(float current);
    void leftVoltageChanged(float voltage);
    void leftPowerChanged(float power);
    void leftSocChanged(float soc);

    // Right Motor UI
    void rightRpmChanged(int rpm);
    void rightCurrentChanged(float current);
    void rightVoltageChanged(float voltage);
    void rightPowerChanged(float power);
    void rightSocChanged(float soc);
    void driveStateChanged(const QString &state);
    void parkEngagedChanged(bool engaged);
    void reverseEnabledChanged(bool enabled);
    void lightsEnabledChanged(bool enabled);
    void lockEnabledChanged(bool enabled);
    void canOnlineChanged(bool online);
    void canStatusChanged(const QString &status);
    void lastControlErrorChanged(const QString &errorMessage);

    // Command path into CANController.
    void parkCommandRequested(bool enabled);
    void directionCommandRequested(bool reverseEnabled);
    void lightsCommandRequested(bool enabled);
    void lockCommandRequested(bool enabled);

public slots:
    // Slots receiving values from CANController
    void updateLeftRpm(int rpm);
    void updateLeftCurrent(float c);
    void updateLeftVoltage(float v);
    void updateLeftPower(float p);
    void updateLeftSoc(float soc);

    void updateRightRpm(int rpm);
    void updateRightCurrent(float c);
    void updateRightVoltage(float v);
    void updateRightPower(float p);
    void updateRightSoc(float soc);

    void updateCanConnection(bool connected);
    void updateCanStatus(const QString &status);
    void updateCanError(const QString &error);

    Q_INVOKABLE void togglePark();
    Q_INVOKABLE void toggleDirection();
    Q_INVOKABLE void toggleLights();
    Q_INVOKABLE void toggleLock();
    Q_INVOKABLE void clearFault();

private slots:
    void onDriveStateLabelChanged(const QString &stateLabel);
    void onTransitionRejected(const QString &reason);
    void onParkChanged(bool engaged);
    void onReverseChanged(bool enabled);

private:
    DriveStateManager driveStateManager;
    int lastLeftRpmValue = 0;
    int lastRightRpmValue = 0;
    bool lightsOn = false;
    bool locked = false;
    bool canConnected = false;
    QString canState = "Disconnected";
    QString controlError = "";
};

#endif 
