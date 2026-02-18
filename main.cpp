#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QProcessEnvironment>
#include <QDebug>

#include "Backend/dashboardcontroller.h"
#include "Backend/cancontroller.h"

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
    // 2. Connect CAN -> DashboardController signals/slots
    //---------------------------------------------------
    QObject::connect(&canController, &CANController::motorRpmUpdated,
                     &dashboardController, &DashboardController::updateRpm);
    QObject::connect(&canController, &CANController::motorSocUpdated,
                     &dashboardController, &DashboardController::updateSoc);
    QObject::connect(&canController, &CANController::motorCurrentUpdated,
                     &dashboardController, &DashboardController::updateCurrent);
    QObject::connect(&canController, &CANController::motorVoltageUpdated,
                     &dashboardController, &DashboardController::updateVoltage);
    QObject::connect(&canController, &CANController::motorPowerUpdated,
                     &dashboardController, &DashboardController::updatePower);

    //---------------------------------------------------
    // 3. Initialize CAN for Pi OS / Boot2Qt
    //---------------------------------------------------
    const QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    const QString canInterface = env.value("DASH_CAN_IFACE", "can0");

    if (!canController.initialize(canInterface)) {
        qWarning() << "CAN init failed on interface" << canInterface;
    }

    //---------------------------------------------------
    // 4. Expose dashboardController to QML
    //---------------------------------------------------
    engine.rootContext()->setContextProperty("dashboardController", &dashboardController);

    //---------------------------------------------------
    // 5. Load QML
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
