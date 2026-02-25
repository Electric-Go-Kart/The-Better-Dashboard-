#ifndef DASHBOARDCONTROLLER_H
#define DASHBOARDCONTROLLER_H

#include <QObject>
#include <QQmlEngine>

class DashboardController : public QObject {
    Q_OBJECT
    QML_ELEMENT

public:
    explicit DashboardController(QObject *parent = nullptr);

signals:
    // Single motor UI signals
    void rpmChanged(int rpm);
    void currentChanged(float current);
    void voltageChanged(float voltage);
    void powerChanged(float power);
    void socChanged(float soc);

public slots:
    // Slots receiving values from CANController
    void updateRpm(int rpm);
    void updateCurrent(float c);
    void updateVoltage(float v);
    void updatePower(float p);
    void updateSoc(float soc);
};

#endif 
