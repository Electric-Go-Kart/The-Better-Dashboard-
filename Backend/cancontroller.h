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
    bool initialize(const QString &interfaceName = "can0"); //vcan0 for testing
    //For testing
    //void start();

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

    void directionChanged(QString direction);

public slots:
    void setDirection(const QString &direction);

private slots:
    void processIncomingFrame();
    //for testing
    //void generateFakeCanData();

private:
    // Create CAN device instance
    QCanBusDevice *device = nullptr;

    // Two motors: left and right
    MotorDataProcessor leftMotor;
    MotorDataProcessor rightMotor;

    // Example CAN IDs — replace with correct ones if needed
    const int LEFT_MOTOR_FRAME_ID  = 0x935; //0x09;
    const int RIGHT_MOTOR_FRAME_ID = 0x902; //0x0A;

    float decodeCurrent(const QByteArray &payload);
    int decodeRpm(const QByteArray &payload);
    float decodeVoltage(const QByteArray &payload);


    int reverse_pin = 26;   // GPIO pin number
    bool locked = false;
    QString direction;


    float deltaTime = 0.02f;   // 20ms update cycle
};

#endif 
