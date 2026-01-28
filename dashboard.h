#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <QHBoxLayout>
#include <QMainWindow>
#include <QPushButton>
#include <QVBoxLayout>
#include "batterypanel.h"
#include "dashboardcontroller.h"
#include "directionpanel.h"
#include "speedometer.h"
#include "test.h"

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
    Test *test;
    DashboardController *controller;
};
#endif // DASHBOARD_H
