#include "dashboardcontroller.h"

DashboardController::DashboardController(QObject *parent)
{
    rpm = 0.0;
    charge = 100;
    direction = "N";
    state = false;
    current = 0.0f;
}

//getter functions
double DashboardController::getSpeed() const
{
    return speed;
}
double DashboardController::getRPM() const
{
    return rpm;
}
int DashboardController::getCharge() const
{
    return charge;
}
QString DashboardController::getDirection() const
{
    return direction ? "Forward" : "Reverse";
    //return direction;
}
bool DashboardController::getState() const
{
    return state;
}
float DashboardController::getCurrent() const
{
    return current;
}

//setter functions (Slots)
void DashboardController::updateSpeed(double rpm)
{
    double speed = rpm / 10;
    emit speedChange(speed);
}
void DashboardController::updateRPM(double rpm) {}
void DashboardController::updateCharge(int charge)
{
    emit batteryChange(charge);
}
void DashboardController::updateDirection()
{
    // toggle state
    direction = !direction;

    // emit signal for the panel to update its button text
    emit directionChange(direction ? "Forward" : "Reverse");
    /*
    if(direction == "Forward")
        direction = "Reverse";
    if(direction == "Reverse")
        direction = "Forward";
    emit directionChange(direction);
*/
}
void DashboardController::updateState(bool state) {}
void DashboardController::updateCurrent(float current) {}
