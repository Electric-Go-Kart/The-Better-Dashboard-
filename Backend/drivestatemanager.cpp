#include "drivestatemanager.h"

#include <QtMath>

DriveStateManager::DriveStateManager(QObject *parent)
    : QObject(parent)
{
}

DriveStateManager::DriveState DriveStateManager::currentState() const
{
    return state;
}

bool DriveStateManager::isParkEngaged() const
{
    return state == DriveState::Park;
}

bool DriveStateManager::isReverseEnabled() const
{
    return state == DriveState::DriveReverse;
}

bool DriveStateManager::isFaulted() const
{
    return state == DriveState::Fault;
}

QString DriveStateManager::stateLabel() const
{
    switch (state) {
    case DriveState::DriveForward:
        return "Drive";
    case DriveState::DriveReverse:
        return "Reverse";
    case DriveState::Park:
        return "Park";
    case DriveState::Fault:
        return "Fault";
    }
    return "Fault";
}

QString DriveStateManager::lastRejectionReason() const
{
    return rejectionReason;
}

bool DriveStateManager::requestPark(bool enabled)
{
    if (state == DriveState::Fault) {
        rejectionReason = "Cannot change Park while faulted";
        emit transitionRejected(rejectionReason);
        return false;
    }

    if (enabled) {
        setState(DriveState::Park);
        return true;
    }

    if (!canExitPark()) {
        rejectionReason = "Cannot exit Park while motor RPM is above safe threshold";
        emit transitionRejected(rejectionReason);
        return false;
    }

    setState(DriveState::DriveForward);
    return true;
}

bool DriveStateManager::requestReverse(bool enabled)
{
    if (state == DriveState::Fault) {
        rejectionReason = "Cannot change direction while faulted";
        emit transitionRejected(rejectionReason);
        return false;
    }
    if (state == DriveState::Park) {
        rejectionReason = "Exit Park before changing direction";
        emit transitionRejected(rejectionReason);
        return false;
    }
    if (enabled && !canEnterReverse()) {
        rejectionReason = "Cannot enter reverse above safe RPM threshold";
        emit transitionRejected(rejectionReason);
        return false;
    }

    setState(enabled ? DriveState::DriveReverse : DriveState::DriveForward);
    return true;
}

void DriveStateManager::setFaulted(bool faulted)
{
    if (faulted) {
        setState(DriveState::Fault);
    } else if (state == DriveState::Fault) {
        setState(DriveState::Park);
    }
}

void DriveStateManager::updateSpeedRpm(int leftRpm, int rightRpm)
{
    lastLeftRpm = leftRpm;
    lastRightRpm = rightRpm;
}

bool DriveStateManager::canExitPark() const
{
    const int maxAbsRpm = qMax(qAbs(lastLeftRpm), qAbs(lastRightRpm));
    return maxAbsRpm <= safeRpmThreshold;
}

bool DriveStateManager::canEnterReverse() const
{
    const int maxAbsRpm = qMax(qAbs(lastLeftRpm), qAbs(lastRightRpm));
    return maxAbsRpm <= safeRpmThreshold;
}

void DriveStateManager::setState(DriveStateManager::DriveState nextState)
{
    if (nextState == state) {
        return;
    }

    state = nextState;
    emit stateChanged(state);
    emit stateLabelChanged(stateLabel());
    emit parkChanged(isParkEngaged());
    emit reverseChanged(isReverseEnabled());
    emit faultChanged(isFaulted());
}
