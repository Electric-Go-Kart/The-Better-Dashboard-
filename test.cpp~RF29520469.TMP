//#if 0
#include "test.h"

Test::Test(QObject *parent)
    : QObject(parent)
    , speed(0.0)
    , charge(100)
    , increase(true)
    , decrease(true)
{
    connect(&speedTimer, &QTimer::timeout, this, &Test::updateSpeed);
    speedTimer.setInterval(500);
    connect(&batteryTimer, &QTimer::timeout, this, &Test::updateBattery);
    batteryTimer.setInterval(1000);
}

void Test::start()
{
    speedTimer.start();
    batteryTimer.start();
}

void Test::updateSpeed()
{
    // oscillate speed between 0 and 1000
    if (increase) {
        speed += 10;
        if (speed >= 1000)
            increase = false;
    } else {
        speed -= 10;
        if (speed <= 0)
            increase = true;
    }
    //qDebug() << "updated:" << speed;
    emit speedChange(speed);
}

void Test::updateBattery()
{
    // oscillate charge between 0 and 100
    if (decrease) {
        charge -= 1;
        if (charge <= 0)
            decrease = false;

    } else {
        charge += 1;
        if (charge >= 100)
            decrease = true;
    }

    emit batteryChange(charge);
}

//#endif
