#include "cancontroller.h"
#include <QDebug>
//for testing
#include <QTimer>
#include <QRandomGenerator>
#include <iostream>

CANController::CANController(QObject *parent)
    : QObject(parent)
{

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
    qDebug() << "CAN initialize called";
    device = QCanBus::instance()->createDevice("socketcan", interfaceName, nullptr);

    if (!device) {
        qWarning() << "Failed to create CAN device!";
        qWarning() << device->errorString();
        return false;
    }

    // Remove bitrate configuration explicitly FOR VCAN TEST ONLY
    device->setConfigurationParameter(QCanBusDevice::ReceiveOwnKey, true);
    device->setConfigurationParameter(QCanBusDevice::LoopbackKey, true);
    device->setConfigurationParameter(QCanBusDevice::BitRateKey, QVariant());



    if (!device->connectDevice()) {
        qWarning() << "Failed to connect to CAN interface!";
        qWarning() << device->errorString();
        return false;
    }

    connect(device, &QCanBusDevice::framesReceived,
            this, &CANController::processIncomingFrame);

    qDebug() << "Signal connected to instance:" << this;

    qDebug() << "Connected to CAN interface:" << interfaceName;

    return true;
}

void CANController::processIncomingFrame()
{
    qDebug() << "CAN processIncomingFrame called";
    while (device->framesAvailable()) {
        QCanBusFrame frame = device->readFrame();
        int id = frame.frameId();
        QByteArray data = frame.payload();


        if (id == LEFT_MOTOR_FRAME_ID) {
            int rpm = decodeRpm(data);
            float current = decodeCurrent(data);
            float voltage = decodeVoltage(data);

        if (processVescFrame(id, payload)) {
            continue;
        }

        processLegacyFrame(id, payload);
    }
}

bool CANController::processLegacyFrame(quint32 frameId, const QByteArray &payload)
{
    if (frameId != static_cast<quint32>(LEGACY_SINGLE_FRAME_ID) || payload.size() < 8) {
        return false;
    }

    const int rpm = readS32BE(payload, 0);
    const float current = static_cast<float>(readS16BE(payload, 4)) / 10.0f;
    const float voltage = static_cast<float>(readS16BE(payload, 6)) / 10.0f;
    lastVoltage = voltage;

    motor.updateValues(rpm, voltage, current, deltaTime);
    return true;
}

bool CANController::processVescFrame(quint32 frameId, const QByteArray &payload)
{
    const int packetId = static_cast<int>((frameId >> 8U) & 0xFFU);
    const int sourceId = static_cast<int>(frameId & 0xFFU);

    if (sourceId != controllerId) {
        return false;
    }

    if (packetId == VESC_STATUS_1_PACKET_ID && payload.size() >= 8) {
        const int rpm = readS32BE(payload, 0);
        const float motorCurrent = static_cast<float>(readS16BE(payload, 4)) / 10.0f;
        const float dutyCycle = static_cast<float>(readS16BE(payload, 6)) / 1000.0f;
        Q_UNUSED(dutyCycle);

        const float currentForModel = qFabs(motorCurrent);
        motor.updateValues(rpm, lastVoltage, currentForModel, deltaTime);
        emit motorCurrentUpdated(motorCurrent);
        return true;
    }

    if (packetId == VESC_STATUS_2_PACKET_ID && payload.size() >= 8) {
        vescAhDischarged = static_cast<float>(readS32BE(payload, 0)) / 10000.0f;
        vescAhCharged = static_cast<float>(readS32BE(payload, 4)) / 10000.0f;

        if (batteryCapacityAh > 0.0f) {
            const float netAh = vescAhDischarged - vescAhCharged;
            const float soc = qBound(0.0f, 100.0f * (1.0f - (netAh / batteryCapacityAh)), 100.0f);
            emit motorSocUpdated(soc);
        }
        return true;
    }

    if (packetId == VESC_STATUS_3_PACKET_ID && payload.size() >= 8) {
        vescWhDischarged = static_cast<float>(readS32BE(payload, 0)) / 10000.0f;
        vescWhCharged = static_cast<float>(readS32BE(payload, 4)) / 10000.0f;
        return true;
    }

    if (packetId == VESC_STATUS_4_PACKET_ID && payload.size() >= 8) {
        const float inputCurrent = static_cast<float>(readS16BE(payload, 4)) / 10.0f;
        emit motorCurrentUpdated(inputCurrent);
        return true;
    }

    if (packetId == VESC_BMS_SOC_PACKET_ID && payload.size() >= 1) {
        hasExplicitSoc = true;
        const float soc = qBound(0.0f, static_cast<float>(static_cast<quint8>(payload[0])), 100.0f);
        emit motorSocUpdated(soc);
        return true;
    }

    // Optional compatibility mode for older payload layouts
    if (packetId == LEGACY_SINGLE_FRAME_ID && payload.size() >= 8) {
        const int rpm = readS32BE(payload, 0);
        const float current = static_cast<float>(readS16BE(payload, 4)) / 10.0f;
        const float voltage = static_cast<float>(readS16BE(payload, 6)) / 10.0f;
        lastVoltage = voltage;
        motor.updateValues(rpm, lastVoltage, current, deltaTime);
        return true;
    }

    return false;
}

qint32 CANController::readS32BE(const QByteArray &payload, int offset)
{
    const qint32 b0 = static_cast<quint8>(payload[offset]);
    const qint32 b1 = static_cast<quint8>(payload[offset + 1]);
    const qint32 b2 = static_cast<quint8>(payload[offset + 2]);
    const qint32 b3 = static_cast<quint8>(payload[offset + 3]);
    return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
}

void CANController::start()
{
    // REMOVE after real CAN hardware is ready
    QTimer *fakeTimer = new QTimer(this);
    connect(fakeTimer, &QTimer::timeout, this, &CANController::generateFakeCanData);
    fakeTimer->start(600); // update every 200ms
}
void CANController::generateFakeCanData()
{
    // Fake Left Motor
//    emit leftMotorRpmUpdated(QRandomGenerator::global()->bounded(0, 6000));
//    emit leftMotorCurrentUpdated(QRandomGenerator::global()->bounded(0, 30) / 1.0f);
    //emit leftVoltageReceived(QRandomGenerator::global()->bounded(40.0f, 60.0f));
    //emit leftPowerReceived(QRandomGenerator::global()->bounded(0.0f, 2000.0f));
//    emit leftMotorSocUpdated(QRandomGenerator::global()->bounded(0, 100) / 1.0f);

    // Fake Right Motor
    //emit rightRpmReceived(QRandomGenerator::global()->bounded(0, 6000));
    //emit rightCurrentReceived(QRandomGenerator::global()->bounded(0.0f, 30.0f));
    //emit rightVoltageReceived(QRandomGenerator::global()->bounded(40.0f, 60.0f));
    //emit rightPowerReceived(QRandomGenerator::global()->bounded(0.0f, 2000.0f));
    //emit rightSocReceived(QRandomGenerator::global()->bounded(0.0f, 100.0f));


    //QCanBusFrame frame(0x123, QByteArray::fromHex("00 00 0B B8 00 69 01 F4"));
    //device->writeFrame(frame);
    //std::cout << "frame = 00 00 0B B8 00 69 01 F4";

    if (!device)
        return;

    if (device->state() != QCanBusDevice::ConnectedState)
        return;

    QCanBusFrame frame(
        0x901,
        QByteArray::fromHex("00000BB8006901F4")
        );

    frame.setExtendedFrameFormat(true);

    //device->writeFrame(frame);
    //qDebug() << "Fake frame sent";

    qint64 result = device->writeFrame(frame);
    qDebug() << "Bytes written:" << result;

    // Optionally try to read back any frames immediately (loopback)
    while (device->framesAvailable()) {
        QCanBusFrame readBack = device->readFrame();
        qDebug() << "Read back frame ID:" << readBack.frameId()
                 << "Payload:" << readBack.payload().toHex();
    }

}
