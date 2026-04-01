#ifndef CANCONTROLLER_H
#define CANCONTROLLER_H

#include <QObject>
#include <QCanBus>
#include <QCanBusDevice>
#include <QTimer>
#include <QString>
#include <QStringList>
#include "motordataprocessor.h"
#include <QQmlEngine>

class CANController : public QObject {
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(bool debugFramesEnabled READ debugFramesEnabled WRITE setDebugFramesEnabled NOTIFY debugFramesEnabledChanged)
    Q_PROPERTY(QString debugFrameLog READ debugFrameLog NOTIFY debugFrameLogChanged)

public:
    explicit CANController(QObject *parent = nullptr);
    bool initialize(const QString &interfaceName = "can0", bool testMode = false);
    void start();
    bool debugFramesEnabled() const;
    QString debugFrameLog() const;

signals:
    // Pass values to DashboardController
    void leftMotorRpmUpdated(int rpm);
    void leftMotorCurrentUpdated(float current);
    void leftMotorVoltageUpdated(float voltage);
    void leftMotorPowerUpdated(float power);
    void leftMotorSocUpdated(float soc);

    void rightMotorRpmUpdated(int rpm);
    void rightMotorCurrentUpdated(float current);
    void rightMotorVoltageUpdated(float voltage);
    void rightMotorPowerUpdated(float power);
    void rightMotorSocUpdated(float soc);
    void canConnectionChanged(bool connected);
    void canStatusChanged(const QString &status);
    void canError(const QString &message);
    void debugFramesEnabledChanged(bool enabled);
    void debugFrameLogChanged(const QString &log);

private slots:
    void processIncomingFrame();
    void handleCanError(QCanBusDevice::CanBusError error);
    void handleCanStateChanged(QCanBusDevice::CanBusDeviceState state);
    void attemptReconnect();
    void sendParkHeartbeat();
    // For test mode only.
    void generateFakeCanData();

public slots:
    void setParkEnabled(bool enabled);
    void setReverseEnabled(bool enabled);
    void setLightsEnabled(bool enabled);
    void setLockEnabled(bool enabled);
    void setDebugFramesEnabled(bool enabled);
    Q_INVOKABLE void clearDebugFrameLog();

private:
    bool connectDevice();
    bool writeFrame(quint32 frameId, const QByteArray &payload, bool extendedFrame = true);
    bool processVescStatusFrame(quint32 frameId, const QByteArray &payload);
    void sendDriveModeFrame();
    void sendBrakeCurrentFrame(float brakeCurrentA);
    void sendZeroCurrentFrame();
    QByteArray encodeCurrentPayload(float currentA) const;
    float activeBrakeCurrentA() const;
    void appendDebugFrame(quint32 frameId, const QByteArray &payload, const QString &note = QString());

    QCanBusDevice *device = nullptr;
    QTimer reconnectTimer;
    QTimer parkHeartbeatTimer;
    bool testModeEnabled = false;

    // Two motors: left and right
    MotorDataProcessor leftMotor;
    MotorDataProcessor rightMotor;

    // Legacy status frame IDs retained as fallbacks for existing hardware setups.
    const quint32 LEGACY_LEFT_MOTOR_FRAME_ID = 0x935;
    const quint32 LEGACY_RIGHT_MOTOR_FRAME_ID = 0x902;

    // VESC style packet IDs packed into the upper 8 bits of EID.
    const quint32 CAN_PACKET_SET_CURRENT = 1;
    const quint32 CAN_PACKET_SET_CURRENT_BRAKE = 2;
    const quint32 CAN_PACKET_STATUS = 9;
    const quint32 APP_MODE_PACKET_ID = 0x7F;
    int leftControllerId = 1;
    int rightControllerId = 2;
    float parkBrakeCurrentA = 12.0f;
    bool parkEnabled = false;
    bool reverseEnabled = false;
    bool lightsEnabled = false;
    bool lockEnabled = false;
    int canBitrate = 500000;
    int reverseGpioPin = 26;
    QString directionState = "forward";

    float decodeCurrent(const QByteArray &payload);
    int decodeRpm(const QByteArray &payload);
    float decodeVoltage(const QByteArray &payload);

    float deltaTime = 0.02f;   // 20ms update cycle
    bool debugFrames = false;
    QStringList debugFrameLines;
    const int maxDebugFrameLines = 18;
};

#endif 
