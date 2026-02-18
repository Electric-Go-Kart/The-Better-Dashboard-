#include "cancontroller.h"
#include <QDebug>
#include <QByteArray>
#include <QCanBusFrame>
#include <QProcessEnvironment>
#include <QtMath>

CANController::CANController(QObject *parent)
    : QObject(parent)
{
    const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    controllerId = env.value("DASH_VESC_CONTROLLER_ID", "53").toInt();
    batteryCapacityAh = env.value("DASH_BATTERY_CAPACITY_AH", "20").toFloat();

    connect(&motor, &MotorDataProcessor::rpmUpdated, this, &CANController::motorRpmUpdated);
    connect(&motor, &MotorDataProcessor::currentUpdated, this, &CANController::motorCurrentUpdated);
    connect(&motor, &MotorDataProcessor::voltageUpdated, this, &CANController::motorVoltageUpdated);
    connect(&motor, &MotorDataProcessor::powerUpdated, this, &CANController::motorPowerUpdated);
    connect(&motor, &MotorDataProcessor::socUpdated, this, [this](float soc) {
        if (!hasExplicitSoc && batteryCapacityAh <= 0.0f) {
            emit motorSocUpdated(soc);
        }
    });
}

bool CANController::initialize(const QString &interfaceName)
{
    if (device) {
        return true;
    }

    device = QCanBus::instance()->createDevice("socketcan", interfaceName, nullptr);

    if (!device) {
        qWarning() << "Failed to create CAN device for interface" << interfaceName;
        return false;
    }

    if (!device->connectDevice()) {
        qWarning() << "Failed to connect to CAN interface" << interfaceName
                   << ":" << device->errorString();
        device->deleteLater();
        device = nullptr;
        return false;
    }

    connect(device, &QCanBusDevice::framesReceived,
            this, &CANController::processIncomingFrame);

    qDebug() << "Connected to CAN interface:" << interfaceName
             << "controllerId:" << controllerId
             << "batteryCapacityAh:" << batteryCapacityAh;
    return true;
}

void CANController::processIncomingFrame()
{
    if (!device) {
        return;
    }

    while (device->framesAvailable()) {
        const QCanBusFrame frame = device->readFrame();
        const quint32 id = frame.frameId();
        const QByteArray payload = frame.payload();

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

qint16 CANController::readS16BE(const QByteArray &payload, int offset)
{
    const qint16 hi = static_cast<quint8>(payload[offset]);
    const qint16 lo = static_cast<quint8>(payload[offset + 1]);
    return static_cast<qint16>((hi << 8) | lo);
}
