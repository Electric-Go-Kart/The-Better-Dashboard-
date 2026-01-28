#include "MotorDataProcessor.h"
#include <algorithm>

MotorDataProcessor::MotorDataProcessor(QObject *parent)
    : QObject(parent)
{
}

void MotorDataProcessor::updateValues(int rpmRaw, float voltageRaw, float currentRaw, float deltaTimeSec)
{
    // Store raw values
    rpm = rpmRaw;
    voltage = voltageRaw;
    current = currentRaw;

    // Send simple signal updates
    emit rpmUpdated(rpm);
    emit voltageUpdated(voltage);
    emit currentUpdated(current);

    // Keep history
    currentHistory.push_back(current);
    if (currentHistory.size() > maxHistorySize)
        currentHistory.pop_front();

    // Compute power (W)
    instantPower = computeInstantPower(voltage, current);
    emit powerUpdated(instantPower);

    // Integrate into total energy (Wh)
    integrateEnergy(instantPower, deltaTimeSec);
    emit powerUpdated(instantPower);

    // Update SOC
    stateOfCharge = computeSOC();
    emit socUpdated(stateOfCharge);
}

float MotorDataProcessor::computeInstantPower(float voltage, float current)
{
    return voltage * current;
}

void MotorDataProcessor::integrateEnergy(float power, float dt)
{
    // power is in watts → multiply by seconds then convert to Wh
    energyUsedWh += (power * dt) / 3600.0f;
}

float MotorDataProcessor::computeSOC()
{
    float soc = 100.0f * (1.0f - (energyUsedWh / batteryCapacityWh));
    return std::clamp(soc, 0.0f, 100.0f);
}
