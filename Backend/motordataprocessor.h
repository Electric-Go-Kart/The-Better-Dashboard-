#ifndef MOTORDATAPROCESSOR_H
#define MOTORDATAPROCESSOR_H

#include <QObject>
#include <deque>

class MotorDataProcessor : public QObject {
    Q_OBJECT

public:
    explicit MotorDataProcessor(QObject *parent = nullptr);

    // Called when new CAN data arrives
    void updateValues(int rpmRaw, float voltageRaw, float currentRaw, float deltaTimeSec);

    // Getters
    float getInstantPower() const { return instantPower; }
    float getEnergyUsedWh() const { return energyUsedWh; }
    float getStateOfCharge() const { return stateOfCharge; }

signals:
    void rpmUpdated(int rpm);
    void currentUpdated(float current);
    void voltageUpdated(float voltage);
    void powerUpdated(float power);
    void socUpdated(float soc);

private:
    // Internal helpers
    float computeInstantPower(float voltage, float current);
    void integrateEnergy(float power, float dt);
    float computeSOC();

    // Configurations
    float batteryCapacityWh = 500.0f;    // example: 500 Wh battery

    // Stored values
    int rpm = 0;
    float voltage = 0.0f;
    float current = 0.0f;

    float instantPower = 0.0f;
    float energyUsedWh = 0.0f;
    float stateOfCharge = 100.0f;

    std::deque<float> currentHistory;
    const size_t maxHistorySize = 200;
};

#endif 
