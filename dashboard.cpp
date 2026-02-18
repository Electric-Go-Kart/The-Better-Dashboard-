#include "dashboard.h"
#include <QApplication>
#include <QScreen>

Dashboard::Dashboard(QWidget *parent)
    : QMainWindow(parent)
{
    // Make window borderless and fullscreen
    setWindowFlags(Qt::FramelessWindowHint);
    showFullScreen();

    // Central widget
    QWidget *central = new QWidget(this);
    setCentralWidget(central);

    // Create panels
    speedometer = new Speedometer(this);
    batteryPanel = new BatteryPanel(this);
    directionPanel = new DirectionPanel(this);
    test = new Test(this);
    controller = new DashboardController(this);

    /* test connections
     *     bool ok =QObject::connect(controller, &DashboardController::rpmChange,
                     speedometer, &Speedometer::setSpeed);
    qDebug() << "Connection success?" << ok;
    */

    // Connect Test signals to DashboardController slots
    QObject::connect(test, &Test::speedChange, controller, &DashboardController::updateSpeed);
    QObject::connect(test, &Test::batteryChange, controller, &DashboardController::updateCharge);

    // Connect DashboardController signals to Speedometer slots
    QObject::connect(controller,
                     &DashboardController::speedChange,
                     speedometer,
                     &Speedometer::setSpeed);
    QObject::connect(controller,
                     &DashboardController::batteryChange,
                     batteryPanel,
                     &BatteryPanel::setBatteryPercent);

    // 2 way connect for QPushButton
    QObject::connect(directionPanel,
                     &DirectionPanel::buttonPressed,
                     controller,
                     &DashboardController::updateDirection);
    QObject::connect(controller,
                     &DashboardController::directionChange,
                     directionPanel,
                     &DirectionPanel::setDirection);

    speedometer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    // Optional close button for touchscreen
    closeButton = new QPushButton("Close", this);
    closeButton->setStyleSheet("font-size: 24px; padding: 10px; background-color: red; color: "
                               "white; border-radius: 10px;");
    connect(closeButton, &QPushButton::clicked, this, &Dashboard::handleClose);

    // Add widgets
    // Speedometer widget
    QHBoxLayout *HSLayout = new QHBoxLayout();
    //HSLayout->addStretch();
    HSLayout->addWidget(speedometer, Qt::AlignCenter); // bigger
    //HSLayout->addStretch();
    HSLayout->addWidget(directionPanel);
    //HSLayout->addStretch();

    // Batterypanel widget
    QHBoxLayout *HBLayout = new QHBoxLayout();
    HBLayout->addStretch();
    HBLayout->addWidget(batteryPanel, 0); // smaller

    // Close button layout
    QHBoxLayout *HELayout = new QHBoxLayout();
    HELayout->addWidget(closeButton, 0);
    HELayout->addStretch();
    //HELayout->addWidget(closeButton, 0, Qt::AlignCenter);

    // Direction Button
    //QHBoxLayout *HDLayout = new QHBoxLayout();
    //HDLayout-> addWidget(directionPanel);

    QVBoxLayout *mainLayout = new QVBoxLayout(central);
    mainLayout->addLayout(HSLayout, 1);
    mainLayout->addLayout(HBLayout);
    mainLayout->addLayout(HELayout);
    mainLayout->setContentsMargins(20, 20, 20, 20);
    mainLayout->setSpacing(20);

    test->start();
}

void Dashboard::handleClose()
{
    QApplication::quit();
}
