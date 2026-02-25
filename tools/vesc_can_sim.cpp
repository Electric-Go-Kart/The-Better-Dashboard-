#include <QCoreApplication>
#include <QCanBus>
#include <QCanBusDevice>
#include <QCanBusFrame>
#include <QDateTime>
#include <QProcessEnvironment>
#include <QTimer>
#include <QDebug>
#include <QScopedPointer>
#include <QtMath>

namespace {
constexpr int kDefaultControllerId = 53;
constexpr int kDefaultIntervalMs = 50;
constexpr float kDefaultNominalVoltage = 51.2f;
constexpr int kStatus1PacketId = 9;
constexpr int kStatus2PacketId = 14;
constexpr int kStatus3PacketId = 15;
constexpr int kStatus4PacketId = 16;
constexpr int kBmsSocPacketId = 38;
}

class VescCanSim : public QObject
{
public:
    explicit VescCanSim(QObject *parent = nullptr)
        : QObject(parent)
    {
        const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
        canInterface = env.value("SIM_CAN_IFACE", "can0");
        controllerId = env.value("SIM_VESC_CONTROLLER_ID", QString::number(kDefaultControllerId)).toInt();
        periodMs = env.value("SIM_PERIOD_MS", QString::number(kDefaultIntervalMs)).toInt();
        nominalVoltage = env.value("SIM_NOMINAL_VOLTAGE", QString::number(kDefaultNominalVoltage)).toFloat();
        batteryCapacityAh = env.value("SIM_BATTERY_CAPACITY_AH", "20").toFloat();
        sendExplicitSoc = env.value("SIM_SEND_EXPLICIT_SOC", "1") != "0";
    }

    bool start()
    {
        device.reset(QCanBus::instance()->createDevice("socketcan", canInterface, nullptr));
        if (!device) {
            qCritical() << "Unable to create socketcan device for" << canInterface;
            return false;
        }

        if (!device->connectDevice()) {
            qCritical() << "Unable to connect to" << canInterface << ":" << device->errorString();
            return false;
        }

        connect(&tickTimer, &QTimer::timeout, this, &VescCanSim::publishFrames);
        tickTimer.start(periodMs);

        qInfo() << "VESC simulator running on" << canInterface
                << "controllerId" << controllerId
                << "periodMs" << periodMs
                << "nominalVoltage" << nominalVoltage
                << "sendExplicitSoc" << sendExplicitSoc;
        return true;
    }

private:
    static QByteArray makeStatus1Payload(int rpm, float motorCurrent, float duty)
    {
        QByteArray p(8, 0);
        const qint32 rpmRaw = rpm;
        const qint16 currentRaw = static_cast<qint16>(motorCurrent * 10.0f);
        const qint16 dutyRaw = static_cast<qint16>(duty * 1000.0f);

        p[0] = static_cast<char>((rpmRaw >> 24) & 0xFF);
        p[1] = static_cast<char>((rpmRaw >> 16) & 0xFF);
        p[2] = static_cast<char>((rpmRaw >> 8) & 0xFF);
        p[3] = static_cast<char>(rpmRaw & 0xFF);
        p[4] = static_cast<char>((currentRaw >> 8) & 0xFF);
        p[5] = static_cast<char>(currentRaw & 0xFF);
        p[6] = static_cast<char>((dutyRaw >> 8) & 0xFF);
        p[7] = static_cast<char>(dutyRaw & 0xFF);
        return p;
    }

    static QByteArray makeStatus2Payload(float ahDischarged, float ahCharged)
    {
        QByteArray p(8, 0);
        const qint32 ahDisRaw = static_cast<qint32>(ahDischarged * 10000.0f);
        const qint32 ahChRaw = static_cast<qint32>(ahCharged * 10000.0f);

        p[0] = static_cast<char>((ahDisRaw >> 24) & 0xFF);
        p[1] = static_cast<char>((ahDisRaw >> 16) & 0xFF);
        p[2] = static_cast<char>((ahDisRaw >> 8) & 0xFF);
        p[3] = static_cast<char>(ahDisRaw & 0xFF);
        p[4] = static_cast<char>((ahChRaw >> 24) & 0xFF);
        p[5] = static_cast<char>((ahChRaw >> 16) & 0xFF);
        p[6] = static_cast<char>((ahChRaw >> 8) & 0xFF);
        p[7] = static_cast<char>(ahChRaw & 0xFF);
        return p;
    }

    static QByteArray makeStatus3Payload(float whDischarged, float whCharged)
    {
        QByteArray p(8, 0);
        const qint32 whDisRaw = static_cast<qint32>(whDischarged * 10000.0f);
        const qint32 whChRaw = static_cast<qint32>(whCharged * 10000.0f);

        p[0] = static_cast<char>((whDisRaw >> 24) & 0xFF);
        p[1] = static_cast<char>((whDisRaw >> 16) & 0xFF);
        p[2] = static_cast<char>((whDisRaw >> 8) & 0xFF);
        p[3] = static_cast<char>(whDisRaw & 0xFF);
        p[4] = static_cast<char>((whChRaw >> 24) & 0xFF);
        p[5] = static_cast<char>((whChRaw >> 16) & 0xFF);
        p[6] = static_cast<char>((whChRaw >> 8) & 0xFF);
        p[7] = static_cast<char>(whChRaw & 0xFF);
        return p;
    }

    static QByteArray makeStatus4Payload(float inputCurrent)
    {
        QByteArray p(8, 0);
        const qint16 currentInRaw = static_cast<qint16>(inputCurrent * 10.0f);
        p[4] = static_cast<char>((currentInRaw >> 8) & 0xFF);
        p[5] = static_cast<char>(currentInRaw & 0xFF);
        return p;
    }

    static QByteArray makeExplicitSocPayload(float socPercent)
    {
        QByteArray p(1, 0);
        const int bounded = qBound(0, static_cast<int>(socPercent + 0.5f), 100);
        p[0] = static_cast<char>(bounded);
        return p;
    }

    quint32 frameIdForPacket(int packetId) const
    {
        return (static_cast<quint32>(packetId) << 8U) | static_cast<quint32>(controllerId & 0xFF);
    }

    bool sendPacket(int packetId, const QByteArray &payload)
    {
        QCanBusFrame frame(frameIdForPacket(packetId), payload);
        frame.setExtendedFrameFormat(true);
        return device->writeFrame(frame);
    }

    void publishFrames()
    {
        const qint64 ms = QDateTime::currentMSecsSinceEpoch();
        const float t = static_cast<float>((ms % 60000) / 1000.0);

        const float phase = (t / 60.0f) * 6.2831853f;
        const float rpmWave = (qSin(phase) + 1.0f) * 0.5f;
        const int rpm = static_cast<int>(rpmWave * 4500.0f);
        const float motorCurrent = rpmWave * 110.0f;
        const float duty = rpm > 0 ? (rpm / 6000.0f) : 0.0f;
        const float inputCurrent = motorCurrent * 0.92f;

        const float dtHours = static_cast<float>(periodMs) / 3600000.0f;
        const float ahStep = inputCurrent * dtHours;
        const float whStep = nominalVoltage * inputCurrent * dtHours;
        ahDischarged += ahStep;
        whDischarged += whStep;

        if (!sendPacket(kStatus1PacketId, makeStatus1Payload(rpm, motorCurrent, duty))) {
            qWarning() << "Failed to write status1 frame:" << device->errorString();
        }
        if (!sendPacket(kStatus2PacketId, makeStatus2Payload(ahDischarged, ahCharged))) {
            qWarning() << "Failed to write status2 frame:" << device->errorString();
        }
        if (!sendPacket(kStatus3PacketId, makeStatus3Payload(whDischarged, whCharged))) {
            qWarning() << "Failed to write status3 frame:" << device->errorString();
        }
        if (!sendPacket(kStatus4PacketId, makeStatus4Payload(inputCurrent))) {
            qWarning() << "Failed to write status4 frame:" << device->errorString();
        }

        if (sendExplicitSoc) {
            float soc = 100.0f;
            if (batteryCapacityAh > 0.0f) {
                soc = 100.0f * (1.0f - (ahDischarged - ahCharged) / batteryCapacityAh);
            }
            if (!sendPacket(kBmsSocPacketId, makeExplicitSocPayload(soc))) {
                qWarning() << "Failed to write explicit SOC frame:" << device->errorString();
            }
        }
    }

    QString canInterface;
    int controllerId = kDefaultControllerId;
    int periodMs = kDefaultIntervalMs;
    float nominalVoltage = kDefaultNominalVoltage;
    float batteryCapacityAh = 20.0f;
    bool sendExplicitSoc = true;
    float ahDischarged = 0.0f;
    float ahCharged = 0.0f;
    float whDischarged = 0.0f;
    float whCharged = 0.0f;
    QTimer tickTimer;
    QScopedPointer<QCanBusDevice> device;
};

int main(int argc, char *argv[])
{
    QCoreApplication app(argc, argv);
    VescCanSim simulator;

    if (!simulator.start()) {
        return 1;
    }

    return app.exec();
}
