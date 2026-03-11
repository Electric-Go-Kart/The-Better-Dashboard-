#include "dashboardcontroller.h"

DashboardController::DashboardController(QObject *parent)
    : QObject(parent)
{
    connect(&driveStateManager, &DriveStateManager::stateLabelChanged,
            this, &DashboardController::onDriveStateLabelChanged);
    connect(&driveStateManager, &DriveStateManager::transitionRejected,
            this, &DashboardController::onTransitionRejected);
    connect(&driveStateManager, &DriveStateManager::parkChanged,
            this, &DashboardController::onParkChanged);
    connect(&driveStateManager, &DriveStateManager::reverseChanged,
            this, &DashboardController::onReverseChanged);
}

QString DashboardController::driveState() const
{
    return driveStateManager.stateLabel();
}

bool DashboardController::parkEngaged() const
{
    return driveStateManager.isParkEngaged();
}

bool DashboardController::reverseEnabled() const
{
    return driveStateManager.isReverseEnabled();
}

bool DashboardController::lightsEnabled() const
{
    return lightsOn;
}

bool DashboardController::lockEnabled() const
{
    return locked;
}

bool DashboardController::canOnline() const
{
    return canConnected;
}

QString DashboardController::canStatus() const
{
    return canState;
}

QString DashboardController::lastControlError() const
{
    return controlError;
}

// LEFT MOTOR ------------------

void DashboardController::updateLeftRpm(int rpm) {
    lastLeftRpmValue = rpm;
    driveStateManager.updateSpeedRpm(lastLeftRpmValue, lastRightRpmValue);
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
    lastRightRpmValue = rpm;
    driveStateManager.updateSpeedRpm(lastLeftRpmValue, lastRightRpmValue);
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

void DashboardController::updateCanConnection(bool connected)
{
    if (canConnected == connected) {
        return;
    }
    canConnected = connected;
    emit canOnlineChanged(canConnected);
    if (!connected) {
        driveStateManager.setFaulted(true);
    }
}

void DashboardController::updateCanStatus(const QString &status)
{
    if (canState == status) {
        return;
    }
    canState = status;
    emit canStatusChanged(canState);
}

void DashboardController::updateCanError(const QString &error)
{
    if (controlError == error) {
        return;
    }
    controlError = error;
    emit lastControlErrorChanged(controlError);
}

void DashboardController::togglePark()
{
    const bool requestedPark = !driveStateManager.isParkEngaged();
    if (!driveStateManager.requestPark(requestedPark)) {
        return;
    }
    emit parkCommandRequested(driveStateManager.isParkEngaged());
}

void DashboardController::toggleDirection()
{
    const bool requestedReverse = !driveStateManager.isReverseEnabled();
    if (!driveStateManager.requestReverse(requestedReverse)) {
        return;
    }
    emit directionCommandRequested(driveStateManager.isReverseEnabled());
}

void DashboardController::toggleLights()
{
    lightsOn = !lightsOn;
    emit lightsEnabledChanged(lightsOn);
    emit lightsCommandRequested(lightsOn);
}

void DashboardController::toggleLock()
{
    locked = !locked;
    emit lockEnabledChanged(locked);
    emit lockCommandRequested(locked);
}

void DashboardController::clearFault()
{
    driveStateManager.setFaulted(false);
    controlError.clear();
    emit lastControlErrorChanged(controlError);
    emit parkCommandRequested(driveStateManager.isParkEngaged());
    emit directionCommandRequested(driveStateManager.isReverseEnabled());
}

void DashboardController::onDriveStateLabelChanged(const QString &stateLabel)
{
    emit driveStateChanged(stateLabel);
}

void DashboardController::onTransitionRejected(const QString &reason)
{
    controlError = reason;
    emit lastControlErrorChanged(controlError);
}

void DashboardController::onParkChanged(bool engaged)
{
    emit parkEngagedChanged(engaged);
}

void DashboardController::onReverseChanged(bool enabled)
{
    emit reverseEnabledChanged(enabled);
}
