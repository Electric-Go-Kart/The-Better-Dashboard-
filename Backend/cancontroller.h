#ifndef CANCONTROLLER_H
#define CANCONTROLLER_H

#include <QObject>
#include <QCanBus>
#include <QCanBusDevice>
#include <QTimer>
#include "motordataprocessor.h"
#include <QQmlEngine>

class CANController : public QObject {
    Q_OBJECT
    QML_ELEMENT

public:
    explicit CANController(QObject *parent = nullptr);
    bool initialize(const QString &interfaceName = "can0", bool testMode = false);
    void start();

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

private:
    bool connectDevice();
    bool writeFrame(quint32 frameId, const QByteArray &payload, bool extendedFrame = true);
    void sendDriveModeFrame();
    void sendBrakeCurrentFrame(float brakeCurrentA);
    void sendZeroCurrentFrame();
    QByteArray encodeCurrentPayload(float currentA) const;
    float activeBrakeCurrentA() const;

    QCanBusDevice *device = nullptr;
    QTimer reconnectTimer;
    QTimer parkHeartbeatTimer;
    bool testModeEnabled = false;

    // Two motors: left and right
    MotorDataProcessor leftMotor;
    MotorDataProcessor rightMotor;

    // VESC style default telemetry IDs (status packets)
    const quint32 LEFT_MOTOR_FRAME_ID = 0x901;
    const quint32 RIGHT_MOTOR_FRAME_ID = 0x902;

    // VESC style packet IDs packed into the upper 8 bits of EID.
    const quint32 CAN_PACKET_SET_CURRENT = 1;
    const quint32 CAN_PACKET_SET_CURRENT_BRAKE = 2;
    const quint32 APP_MODE_PACKET_ID = 0x7F;
    int leftControllerId = 1;
    int rightControllerId = 2;
    float parkBrakeCurrentA = 12.0f;
    bool parkEnabled = false;
    bool reverseEnabled = false;
    bool lightsEnabled = false;
    bool lockEnabled = false;

    float decodeCurrent(const QByteArray &payload);
    int decodeRpm(const QByteArray &payload);
    float decodeVoltage(const QByteArray &payload);

    float deltaTime = 0.02f;   // 20ms update cycle
};

#endif 
