#include "DashboardController.h"

DashboardController::DashboardController(QObject *parent)
    : QObject(parent)
{
}

// LEFT MOTOR ------------------

void DashboardController::updateLeftRpm(int rpm) {
    emit leftRpmChanged(rpm);
}

void DashboardController::updateLeftCurrent(float c) {
    emit leftCurrentChanged(c);
}

void DashboardController::updateLeftVoltage(float v) {
    emit leftVoltageChanged(v);
}

void DashboardController::updateLeftPower(float p) {
    emit leftPowerChanged(p);
}

void DashboardController::updateLeftSoc(float soc) {
    emit leftSocChanged(soc);
}


// RIGHT MOTOR ------------------

void DashboardController::updateRightRpm(int rpm) {
    emit rightRpmChanged(rpm);
}

void DashboardController::updateRightCurrent(float c) {
    emit rightCurrentChanged(c);
}

void DashboardController::updateRightVoltage(float v) {
    emit rightVoltageChanged(v);
}

void DashboardController::updateRightPower(float p) {
    emit rightPowerChanged(p);
}

void DashboardController::updateRightSoc(float soc) {
    emit rightSocChanged(soc);
}
