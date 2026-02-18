#include "dashboardcontroller.h"

DashboardController::DashboardController(QObject *parent)
    : QObject(parent)
{
}

void DashboardController::updateRpm(int rpm) {
    emit rpmChanged(rpm);
}

void DashboardController::updateCurrent(float c) {
    emit currentChanged(c);
}

void DashboardController::updateVoltage(float v) {
    emit voltageChanged(v);
}

void DashboardController::updatePower(float p) {
    emit powerChanged(p);
}

void DashboardController::updateSoc(float soc) {
    emit socChanged(soc);
}
