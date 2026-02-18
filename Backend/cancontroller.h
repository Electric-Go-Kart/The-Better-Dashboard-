#ifndef CANCONTROLLER_H
#define CANCONTROLLER_H

#include <QObject>
#include <QCanBus>
#include <QCanBusDevice>
#include <QtGlobal>
#include "motordataprocessor.h"

class CANController : public QObject {
    Q_OBJECT

public:
    explicit CANController(QObject *parent = nullptr);
    bool initialize(const QString &interfaceName = "can0");

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

    MotorDataProcessor motor;

    // Defaults for a single FSESC motor setup.
    int controllerId = 53;
    const int LEGACY_SINGLE_FRAME_ID = 0x09;
    const int VESC_STATUS_1_PACKET_ID = 9;
    const int VESC_STATUS_2_PACKET_ID = 14;
    const int VESC_STATUS_3_PACKET_ID = 15;
    const int VESC_STATUS_4_PACKET_ID = 16;
    const int VESC_BMS_SOC_PACKET_ID = 38;

    bool processLegacyFrame(quint32 frameId, const QByteArray &payload);
    bool processVescFrame(quint32 frameId, const QByteArray &payload);
    static qint32 readS32BE(const QByteArray &payload, int offset);
    static qint16 readS16BE(const QByteArray &payload, int offset);

    float deltaTime = 0.02f;
    float lastVoltage = 0.0f;
    float batteryCapacityAh = 0.0f;
    bool hasExplicitSoc = false;
    float vescAhDischarged = 0.0f;
    float vescAhCharged = 0.0f;
    float vescWhDischarged = 0.0f;
    float vescWhCharged = 0.0f;
};

#endif 
