#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

#include "Backend/DashboardController.h"
#include "Backend/CANController.h"

int main(int argc, char *argv[])
{
    qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));

    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;

    //---------------------------------------------------
    // 1. Create backend classes
    //---------------------------------------------------
    DashboardController dashboardController;
    CANController canController;

    //---------------------------------------------------
    // 2. Connect CAN → DashboardController signals/slots
    //---------------------------------------------------
    //LEFT MOTOR
    QObject::connect(&canController, &CANController::leftMotorRpmUpdated,
                     &dashboardController, &DashboardController::updateLeftRpm);

    QObject::connect(&canController, &CANController::leftMotorSocUpdated,
                     &dashboardController, &DashboardController::updateLeftSoc);

    QObject::connect(&canController, &CANController::leftMotorCurrentUpdated,
                     &dashboardController, &DashboardController::updateLeftCurrent);

    QObject::connect(&canController, &CANController::leftMotorVoltageUpdated,
                     &dashboardController, &DashboardController::updateLeftVoltage);

    QObject::connect(&canController, &CANController::leftMotorPowerUpdated,
                     &dashboardController, &DashboardController::updateLeftPower);


    //RIGHT MOTOR
    QObject::connect(&canController, &CANController::rightMotorRpmUpdated,
                     &dashboardController, &DashboardController::updateRightRpm);

    QObject::connect(&canController, &CANController::rightMotorSocUpdated,
                     &dashboardController, &DashboardController::updateRightSoc);

    QObject::connect(&canController, &CANController::rightMotorCurrentUpdated,
                     &dashboardController, &DashboardController::updateRightCurrent);

    QObject::connect(&canController, &CANController::rightMotorVoltageUpdated,
                     &dashboardController, &DashboardController::updateRightVoltage);

    QObject::connect(&canController, &CANController::rightMotorPowerUpdated,
                     &dashboardController, &DashboardController::updateRightPower);

    // (Add others if you want current/voltage/power)

    //---------------------------------------------------
    // 3. Expose dashboardController to QML
    //---------------------------------------------------
    engine.rootContext()->setContextProperty("dashboardController", &dashboardController);

    //---------------------------------------------------
    // 4. Load QML
    //---------------------------------------------------
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    engine.loadFromModule("QMLCDash", "Main");

    return app.exec();
}



/*
#include <QGuiApplication>
#include <QQmlApplicationEngine>

int main(int argc, char *argv[])
{
    qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));

    // Creates main application object
    QGuiApplication app(argc, argv);

    // Loads and runs QML files
    QQmlApplicationEngine engine;

    // Error handling
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreationFailed,
        &app,
        []() { QCoreApplication::exit(-1); },
        Qt::QueuedConnection);

    // Loads Main.qml
    engine.loadFromModule("QMLCDash", "Main");

    // Start Qt event loop
    return app.exec();
}
*/
