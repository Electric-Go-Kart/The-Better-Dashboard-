#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include "speedometer.h"
#include "batterypanel.h"
#include "test.h"
#include "dashboardcontroller.h"
#include "directionpanel.h"

class Dashboard : public QMainWindow
{
    Q_OBJECT
public:
    explicit Dashboard(QWidget *parent = nullptr);

private slots:
    void handleClose();

private:
    Speedometer *speedometer;
    BatteryPanel *batteryPanel;
    QPushButton *closeButton;
    DirectionPanel *directionPanel;
    Test* test;
    DashboardController* controller;
};
#endif // DASHBOARD_H
