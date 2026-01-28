#ifndef DASHBOARDCONTROLLER_H
#define DASHBOARDCONTROLLER_H

#include <QObject>
#include <QQmlEngine>
#include <QString>
#include "test.h"

class DashboardController : public QObject
{
    Q_OBJECT
    QML_ELEMENT

    //getters for QML
    //Q_PROPERTY(double rpm READ rpm WRITE setRpm NOTIFY rpmChanged)
    //Q_PROPERTY(int charge READ charge WRITE setCharge NOTIFY chargeChanged)
    //Q_PROPERTY(QString direction READ direction WRITE setDirection NOTIFY directionChanged)
    //Q_PROPERTY(bool state READ state WRITE setState NOTIFY stateChanged)
    //Q_PROPERTY(float current READ current WRITE setCurrent NOTIFY currentChanged)

public:
    explicit DashboardController(QObject *parent = nullptr);

    //getters for C++
    double getSpeed() const;
    double getRPM() const;
    int getCharge() const;
    QString getDirection() const;
    bool getState() const;
    float getCurrent() const;

signals:
    void speedChange(double speed);
    void rpmChange(double rpm);
    void batteryChange(int charge);
    void directionChange(QString direction);
    void stateChange(bool state);
    void currentChange(float current);
public slots:
    void updateSpeed(double speed);
    void updateRPM(double rpm);
    void updateCharge(int charge);
    void updateDirection();
    //void updateDirection(QString direction);
    void updateState(bool state);
    void updateCurrent(float current);

private:
    double speed;
    double rpm;
    int charge;
    bool direction;
    bool state;
    float current;
};

#endif // DASHBOARDCONTROLLER_H
