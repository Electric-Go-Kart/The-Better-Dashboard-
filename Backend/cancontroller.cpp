#include "cancontroller.h"
#include <QDebug>
#include <QByteArray>
#include <QCoreApplication>
#include <QProcessEnvironment>
#include <QTimer>
#include <QRandomGenerator>

namespace {
quint32 makeVescFrameId(quint32 packetId, int controllerId)
{
    return ((packetId & 0xFFU) << 8U) | static_cast<quint32>(controllerId & 0xFF);
}
}

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

    reconnectTimer.setInterval(1000);
    reconnectTimer.setSingleShot(false);
    connect(&reconnectTimer, &QTimer::timeout, this, &CANController::attemptReconnect);

    parkHeartbeatTimer.setInterval(150);
    parkHeartbeatTimer.setSingleShot(false);
    connect(&parkHeartbeatTimer, &QTimer::timeout, this, &CANController::sendParkHeartbeat);
}

bool CANController::initialize(const QString &interfaceName, bool testMode)
{
    testModeEnabled = testMode;
    leftControllerId = qEnvironmentVariableIntValue("DASH_LEFT_CONTROLLER_ID");
    rightControllerId = qEnvironmentVariableIntValue("DASH_RIGHT_CONTROLLER_ID");
    if (leftControllerId == 0) {
        leftControllerId = 1;
    }
    if (rightControllerId == 0) {
        rightControllerId = 2;
    }
    bool ok = false;
    const float envBrake = qEnvironmentVariable("DASH_PARK_BRAKE_CURRENT_A").toFloat(&ok);
    if (ok && envBrake > 0.0f) {
        parkBrakeCurrentA = envBrake;
    }

    if (device) {
        device->disconnectDevice();
        device->deleteLater();
        device = nullptr;
    }

    device = QCanBus::instance()->createDevice("socketcan", interfaceName, nullptr);
    if (!device) {
        emit canConnectionChanged(false);
        emit canStatusChanged("Disconnected");
        emit canError("Failed to create socketcan device for " + interfaceName);
        qWarning() << "Failed to create CAN device for interface" << interfaceName;
        return false;
    }

    device->setConfigurationParameter(QCanBusDevice::ReceiveOwnKey, testModeEnabled);
    device->setConfigurationParameter(QCanBusDevice::LoopbackKey, testModeEnabled);

    connect(device, &QCanBusDevice::framesReceived, this, &CANController::processIncomingFrame);
    connect(device, &QCanBusDevice::errorOccurred, this, &CANController::handleCanError);
    connect(device, &QCanBusDevice::stateChanged, this, &CANController::handleCanStateChanged);

    const bool connected = connectDevice();
    if (!connected) {
        reconnectTimer.start();
    }
    return connected;
}

bool CANController::connectDevice()
{
    if (!device) {
        return false;
    }

    if (device->connectDevice()) {
        emit canConnectionChanged(true);
        emit canStatusChanged("Connected");
        qDebug() << "Connected to CAN interface";
        reconnectTimer.stop();
        return true;
    }

    const QString errorText = device->errorString();
    emit canConnectionChanged(false);
    emit canStatusChanged("Disconnected");
    emit canError("CAN connection failed: " + errorText);
    qWarning() << "Failed to connect CAN interface:" << errorText;
    return false;
}

void CANController::attemptReconnect()
{
    if (!device) {
        return;
    }
    if (device->state() == QCanBusDevice::ConnectedState) {
        reconnectTimer.stop();
        return;
    }
    connectDevice();
}

void CANController::processIncomingFrame()
{
    if (!device) {
        return;
    }

    while (device->framesAvailable()) {
        const QCanBusFrame frame = device->readFrame();
        const quint32 id = frame.frameId();
        const QByteArray data = frame.payload();
        if (data.size() < 8) {
            qWarning() << "Skipping short CAN frame id" << Qt::hex << id << "dlc" << data.size();
            continue;
        }

        if (id == LEFT_MOTOR_FRAME_ID) {
            const int rpm = decodeRpm(data);
            const float current = decodeCurrent(data);
            const float voltage = decodeVoltage(data);

            leftMotor.updateValues(rpm, voltage, current, deltaTime);
        }

        else if (id == RIGHT_MOTOR_FRAME_ID) {
            const int rpm = decodeRpm(data);
            const float current = decodeCurrent(data);
            const float voltage = decodeVoltage(data);

            rightMotor.updateValues(rpm, voltage, current, deltaTime);
        }
    }
}

// ---------------- DECODING FUNCTIONS ------------------

int CANController::decodeRpm(const QByteArray &p)
{
    // Bytes 0–3 : Big-endian RPM
    const quint8 b0 = static_cast<quint8>(p[0]);
    const quint8 b1 = static_cast<quint8>(p[1]);
    const quint8 b2 = static_cast<quint8>(p[2]);
    const quint8 b3 = static_cast<quint8>(p[3]);
    const qint32 raw = (static_cast<qint32>(b0) << 24)
                       | (static_cast<qint32>(b1) << 16)
                       | (static_cast<qint32>(b2) << 8)
                       | static_cast<qint32>(b3);
    return raw; // already scaled
}

float CANController::decodeCurrent(const QByteArray &p)
{
    // Bytes 4–5 : int16 / 10
    const quint8 b4 = static_cast<quint8>(p[4]);
    const quint8 b5 = static_cast<quint8>(p[5]);
    const qint16 raw = static_cast<qint16>((static_cast<quint16>(b4) << 8) | b5);
    return raw / 10.0f;
}

float CANController::decodeVoltage(const QByteArray &p)
{
    // Bytes 6–7 : voltage * 10 (status packet style)
    const quint8 b6 = static_cast<quint8>(p[6]);
    const quint8 b7 = static_cast<quint8>(p[7]);
    const qint16 raw = static_cast<qint16>((static_cast<quint16>(b6) << 8) | b7);
    return raw / 10.0f;
}

void CANController::start()
{
    if (!testModeEnabled) {
        return;
    }
    QTimer *fakeTimer = new QTimer(this);
    connect(fakeTimer, &QTimer::timeout, this, &CANController::generateFakeCanData);
    fakeTimer->start(600);
}

void CANController::generateFakeCanData()
{
    if (!device)
        return;

    if (device->state() != QCanBusDevice::ConnectedState)
        return;

    const int rpm = QRandomGenerator::global()->bounded(900, 3000);
    const int currentDeciAmp = QRandomGenerator::global()->bounded(20, 120);
    const int voltageDeciVolt = QRandomGenerator::global()->bounded(450, 560);
    QByteArray payload(8, 0x00);
    payload[0] = static_cast<char>((rpm >> 24) & 0xFF);
    payload[1] = static_cast<char>((rpm >> 16) & 0xFF);
    payload[2] = static_cast<char>((rpm >> 8) & 0xFF);
    payload[3] = static_cast<char>(rpm & 0xFF);
    payload[4] = static_cast<char>((currentDeciAmp >> 8) & 0xFF);
    payload[5] = static_cast<char>(currentDeciAmp & 0xFF);
    payload[6] = static_cast<char>((voltageDeciVolt >> 8) & 0xFF);
    payload[7] = static_cast<char>(voltageDeciVolt & 0xFF);

    QCanBusFrame frame(
        LEFT_MOTOR_FRAME_ID,
        payload);

    frame.setExtendedFrameFormat(true);
    device->writeFrame(frame);
}

void CANController::handleCanError(QCanBusDevice::CanBusError error)
{
    if (error == QCanBusDevice::NoError || !device) {
        return;
    }
    const QString message = device->errorString();
    emit canError(message);
    emit canStatusChanged("Error");
    emit canConnectionChanged(false);
    reconnectTimer.start();
}

void CANController::handleCanStateChanged(QCanBusDevice::CanBusDeviceState state)
{
    const bool connected = (state == QCanBusDevice::ConnectedState);
    emit canConnectionChanged(connected);
    emit canStatusChanged(connected ? "Connected" : "Disconnected");
    if (!connected) {
        reconnectTimer.start();
    }
}

void CANController::setParkEnabled(bool enabled)
{
    parkEnabled = enabled;
    if (parkEnabled) {
        sendParkHeartbeat();
        if (!parkHeartbeatTimer.isActive()) {
            parkHeartbeatTimer.start();
        }
    } else {
        parkHeartbeatTimer.stop();
        sendBrakeCurrentFrame(0.0f);
        sendZeroCurrentFrame();
    }
    sendDriveModeFrame();
}

void CANController::setReverseEnabled(bool enabled)
{
    reverseEnabled = enabled;
    sendDriveModeFrame();
}

void CANController::setLightsEnabled(bool enabled)
{
    lightsEnabled = enabled;
    sendDriveModeFrame();
}

void CANController::setLockEnabled(bool enabled)
{
    lockEnabled = enabled;
    sendDriveModeFrame();
}

void CANController::sendParkHeartbeat()
{
    if (!parkEnabled) {
        return;
    }
    sendBrakeCurrentFrame(activeBrakeCurrentA());
    sendZeroCurrentFrame();
}

bool CANController::writeFrame(quint32 frameId, const QByteArray &payload, bool extendedFrame)
{
    if (!device || device->state() != QCanBusDevice::ConnectedState) {
        return false;
    }
    QCanBusFrame frame(frameId, payload);
    frame.setExtendedFrameFormat(extendedFrame);
    const bool ok = device->writeFrame(frame);
    if (!ok) {
        emit canError("Failed TX frame 0x" + QString::number(frameId, 16) + ": " + device->errorString());
    }
    return ok;
}

QByteArray CANController::encodeCurrentPayload(float currentA) const
{
    const qint32 milliAmp = static_cast<qint32>(currentA * 1000.0f);
    QByteArray payload(4, 0x00);
    payload[0] = static_cast<char>((milliAmp >> 24) & 0xFF);
    payload[1] = static_cast<char>((milliAmp >> 16) & 0xFF);
    payload[2] = static_cast<char>((milliAmp >> 8) & 0xFF);
    payload[3] = static_cast<char>(milliAmp & 0xFF);
    return payload;
}

void CANController::sendBrakeCurrentFrame(float brakeCurrentA)
{
    const QByteArray payload = encodeCurrentPayload(brakeCurrentA);
    writeFrame(makeVescFrameId(CAN_PACKET_SET_CURRENT_BRAKE, leftControllerId), payload, true);
    writeFrame(makeVescFrameId(CAN_PACKET_SET_CURRENT_BRAKE, rightControllerId), payload, true);
}

void CANController::sendZeroCurrentFrame()
{
    const QByteArray payload = encodeCurrentPayload(0.0f);
    writeFrame(makeVescFrameId(CAN_PACKET_SET_CURRENT, leftControllerId), payload, true);
    writeFrame(makeVescFrameId(CAN_PACKET_SET_CURRENT, rightControllerId), payload, true);
}

void CANController::sendDriveModeFrame()
{
    QByteArray payload(8, 0x00);
    payload[0] = static_cast<char>(parkEnabled ? 1 : 0);
    payload[1] = static_cast<char>(reverseEnabled ? 1 : 0);
    payload[2] = static_cast<char>(lockEnabled ? 1 : 0);
    payload[3] = static_cast<char>(lightsEnabled ? 1 : 0);
    writeFrame(makeVescFrameId(APP_MODE_PACKET_ID, leftControllerId), payload, true);
}

float CANController::activeBrakeCurrentA() const
{
    return parkBrakeCurrentA;
}
