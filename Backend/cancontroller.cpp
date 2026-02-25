#include "CANController.h"
#include <QDebug>
//for testing
#include <QTimer>
#include <QRandomGenerator>

CANController::CANController(QObject *parent)
    : QObject(parent)
{
    // REMOVE after real CAN hardware is ready
    QTimer *fakeTimer = new QTimer(this);
    connect(fakeTimer, &QTimer::timeout, this, &CANController::generateFakeCanData);
    fakeTimer->start(600); // update every 200ms

    // Connect Motor Processors to outgoing signals
    connect(&leftMotor, &MotorDataProcessor::rpmUpdated,
            this, &CANController::leftMotorRpmUpdated);

    connect(&leftMotor, &MotorDataProcessor::currentUpdated,
            this, &CANController::leftMotorCurrentUpdated);

    connect(&leftMotor, &MotorDataProcessor::voltageUpdated,
            this, &CANController::leftMotorVoltageUpdated);

    connect(&leftMotor, &MotorDataProcessor::powerUpdated,
            this, &CANController::leftMotorPowerUpdated);

    connect(&leftMotor, &MotorDataProcessor::socUpdated,
            this, &CANController::leftMotorSocUpdated);

    connect(&rightMotor, &MotorDataProcessor::rpmUpdated,
            this, &CANController::rightMotorRpmUpdated);

    connect(&rightMotor, &MotorDataProcessor::currentUpdated,
            this, &CANController::rightMotorCurrentUpdated);

    connect(&rightMotor, &MotorDataProcessor::voltageUpdated,
            this, &CANController::rightMotorVoltageUpdated);

    connect(&rightMotor, &MotorDataProcessor::powerUpdated,
            this, &CANController::rightMotorPowerUpdated);

    connect(&rightMotor, &MotorDataProcessor::socUpdated,
            this, &CANController::rightMotorSocUpdated);
}

bool CANController::initialize(const QString &interfaceName)
{
    device = QCanBus::instance()->createDevice("socketcan", interfaceName, nullptr);

    if (!device) {
        qWarning() << "Failed to create CAN device!";
        return false;
    }

    if (!device->connectDevice()) {
        qWarning() << "Failed to connect to CAN interface!";
        return false;
    }

    connect(device, &QCanBusDevice::framesReceived,
            this, &CANController::processIncomingFrame);

    qDebug() << "Connected to CAN interface:" << interfaceName;
    return true;
}

void CANController::processIncomingFrame()
{
    while (device->framesAvailable()) {
        QCanBusFrame frame = device->readFrame();
        int id = frame.frameId();
        QByteArray data = frame.payload();

        if (id == LEFT_MOTOR_FRAME_ID) {
            int rpm = decodeRpm(data);
            float current = decodeCurrent(data);
            float voltage = decodeVoltage(data);

            leftMotor.updateValues(rpm, voltage, current, deltaTime);
        }

        else if (id == RIGHT_MOTOR_FRAME_ID) {
            int rpm = decodeRpm(data);
            float current = decodeCurrent(data);
            float voltage = decodeVoltage(data);

            rightMotor.updateValues(rpm, voltage, current, deltaTime);
        }
    }
}

// ---------------- DECODING FUNCTIONS ------------------

int CANController::decodeRpm(const QByteArray &p)
{
    // Bytes 0–3 : Big-endian RPM
    int raw = (p[0] << 24) | (p[1] << 16) | (p[2] << 8) | (p[3]);
    return raw; // already scaled
}

float CANController::decodeCurrent(const QByteArray &p)
{
    // Bytes 4–5 : int16 / 10
    short raw = (p[4] << 8) | (p[5]);
    return raw / 10.0f;
}

float CANController::decodeVoltage(const QByteArray &p)
{
    // Example only — adjust depending on your real VESC protocol
    // Bytes 6–7 : voltage * 10
    short raw = (p[6] << 8) | (p[7]);
    return raw / 10.0f;
}

void CANController::generateFakeCanData()
{
    // Fake Left Motor
    emit leftMotorRpmUpdated(QRandomGenerator::global()->bounded(0, 6000));
    emit leftMotorCurrentUpdated(QRandomGenerator::global()->bounded(0, 30) / 1.0f);
    //emit leftVoltageReceived(QRandomGenerator::global()->bounded(40.0f, 60.0f));
    //emit leftPowerReceived(QRandomGenerator::global()->bounded(0.0f, 2000.0f));
    emit leftMotorSocUpdated(QRandomGenerator::global()->bounded(0, 100) / 1.0f);

    // Fake Right Motor
    //emit rightRpmReceived(QRandomGenerator::global()->bounded(0, 6000));
    //emit rightCurrentReceived(QRandomGenerator::global()->bounded(0.0f, 30.0f));
    //emit rightVoltageReceived(QRandomGenerator::global()->bounded(40.0f, 60.0f));
    //emit rightPowerReceived(QRandomGenerator::global()->bounded(0.0f, 2000.0f));
    //emit rightSocReceived(QRandomGenerator::global()->bounded(0.0f, 100.0f));
}
