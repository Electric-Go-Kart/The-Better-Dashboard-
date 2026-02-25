#ifndef DASHBOARDCONTROLLER_H
#define DASHBOARDCONTROLLER_H

#include <QObject>

class DashboardController : public QObject {
    Q_OBJECT

public:
    explicit DashboardController(QObject *parent = nullptr);

signals:
    // Left Motor UI
    void leftRpmChanged(int rpm);
    void leftCurrentChanged(float current);
    void leftVoltageChanged(float voltage);
    void leftPowerChanged(float power);
    void leftSocChanged(float soc);

    // Right Motor UI
    void rightRpmChanged(int rpm);
    void rightCurrentChanged(float current);
    void rightVoltageChanged(float voltage);
    void rightPowerChanged(float power);
    void rightSocChanged(float soc);

public slots:
    // Slots receiving values from CANController
    void updateLeftRpm(int rpm);
    void updateLeftCurrent(float c);
    void updateLeftVoltage(float v);
    void updateLeftPower(float p);
    void updateLeftSoc(float soc);

    void updateRightRpm(int rpm);
    void updateRightCurrent(float c);
    void updateRightVoltage(float v);
    void updateRightPower(float p);
    void updateRightSoc(float soc);
};

#endif 
