#ifndef CANCONTROLLER_H
#define CANCONTROLLER_H

#include <QObject>
#include <QCanBus>
#include <QCanBusDevice>
#include "MotorDataProcessor.h"

class CANController : public QObject {
    Q_OBJECT

public:
    explicit CANController(QObject *parent = nullptr);
    bool initialize(const QString &interfaceName = "vcan0");

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

private slots:
    void processIncomingFrame();
    //for testing
    void generateFakeCanData();

private:
    QCanBusDevice *device = nullptr;

    // Two motors: left and right
    MotorDataProcessor leftMotor;
    MotorDataProcessor rightMotor;

    // Example CAN IDs — replace with correct ones if needed
    const int LEFT_MOTOR_FRAME_ID  = 0x09;
    const int RIGHT_MOTOR_FRAME_ID = 0x0A;

    float decodeCurrent(const QByteArray &payload);
    int decodeRpm(const QByteArray &payload);
    float decodeVoltage(const QByteArray &payload);

    float deltaTime = 0.02f;   // 20ms update cycle
};

#endif 
