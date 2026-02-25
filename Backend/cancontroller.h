#ifndef CANCONTROLLER_H
#define CANCONTROLLER_H

#include <QObject>
#include <QCanBus>
#include <QCanBusDevice>
#include "MotorDataProcessor.h"
#include <QQmlEngine>

class CANController : public QObject {
    Q_OBJECT
    QML_ELEMENT

public:
    explicit CANController(QObject *parent = nullptr);
    bool initialize(const QString &interfaceName = "vcan0");
    void start();

signals:
    // Single motor values published to DashboardController
    void motorRpmUpdated(int rpm);
    void motorCurrentUpdated(float current);
    void motorVoltageUpdated(float voltage);
    void motorPowerUpdated(float power);
    void motorSocUpdated(float soc);

private slots:
    void processIncomingFrame();

private:
    QCanBusDevice *device = nullptr;
    QCanBusDevice *device2 = nullptr;

    // Two motors: left and right
    MotorDataProcessor leftMotor;
    MotorDataProcessor rightMotor;

    // Example CAN IDs — replace with correct ones if needed
    const int LEFT_MOTOR_FRAME_ID  = 0x901; //0x09;
    const int RIGHT_MOTOR_FRAME_ID = 0x902; //0x0A;

    float decodeCurrent(const QByteArray &payload);
    int decodeRpm(const QByteArray &payload);
    float decodeVoltage(const QByteArray &payload);

    float deltaTime = 0.02f;   // 20ms update cycle
};

#endif 
