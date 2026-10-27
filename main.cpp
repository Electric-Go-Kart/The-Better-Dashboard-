#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QProcessEnvironment>
#include <QFont>

#include "Backend/dashboardcontroller.h"
#include "Backend/cancontroller.h"
#include "Backend/camerafeed.h"
#include "Backend/lidarfeed.h"
#include "Backend/stl27lreader.h"
#include "Backend/detectionmodel.h"
#include "Backend/detectionfeed.h"

int main(int argc, char *argv[])
{
    qputenv("QT_IM_MODULE", QByteArray("qtvirtualkeyboard"));

    QGuiApplication app(argc, argv);
    QFont dashFont("Noto Sans");
    dashFont.setStyleHint(QFont::SansSerif);
    app.setFont(dashFont);
    QQmlApplicationEngine engine;

    //---------------------------------------------------
    // 1. Create backend classes
    //---------------------------------------------------
    DashboardController dashboardController;
    CANController canController;
    const QString interfaceName = qEnvironmentVariable("DASH_CAN_INTERFACE", "can0");
    const bool testMode = qEnvironmentVariable("DASH_CAN_TEST_MODE", "0") == "1";
    canController.initialize(interfaceName, testMode);
    canController.start();

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

    QObject::connect(&canController, &CANController::canConnectionChanged,
                     &dashboardController, &DashboardController::updateCanConnection);
    QObject::connect(&canController, &CANController::canStatusChanged,
                     &dashboardController, &DashboardController::updateCanStatus);
    QObject::connect(&canController, &CANController::canError,
                     &dashboardController, &DashboardController::updateCanError);

    QObject::connect(&dashboardController, &DashboardController::parkCommandRequested,
                     &canController, &CANController::setParkEnabled);
    QObject::connect(&dashboardController, &DashboardController::directionCommandRequested,
                     &canController, &CANController::setReverseEnabled);
    QObject::connect(&dashboardController, &DashboardController::lightsCommandRequested,
                     &canController, &CANController::setLightsEnabled);
    QObject::connect(&dashboardController, &DashboardController::lockCommandRequested,
                     &canController, &CANController::setLockEnabled);

    //---------------------------------------------------
    // 3. Expose dashboardController to QML
    //---------------------------------------------------
    engine.rootContext()->setContextProperty("dashboardController", &dashboardController);
    engine.rootContext()->setContextProperty("canController", &canController);
    const bool uiSweepTestEnabled = qEnvironmentVariable("DASH_UI_SWEEP_TEST", "0") == "1";
    engine.rootContext()->setContextProperty("uiSweepTestEnabled", uiSweepTestEnabled);

    CameraFeed cameraFeed;
    cameraFeed.listen(qEnvironmentVariableIntValue("DASH_CAMERA_PORT") > 0
                          ? quint16(qEnvironmentVariableIntValue("DASH_CAMERA_PORT"))
                          : quint16(5600));
    engine.addImageProvider("kartcam", new CameraImageProvider(cameraFeed.store()));
    engine.rootContext()->setContextProperty("cameraFeed", &cameraFeed);

    LidarFeed lidarFeed;
    lidarFeed.listen(qEnvironmentVariableIntValue("DASH_LIDAR_PORT") > 0
                         ? quint16(qEnvironmentVariableIntValue("DASH_LIDAR_PORT"))
                         : quint16(5601));
    engine.rootContext()->setContextProperty("lidarFeed", &lidarFeed);

    // DASH_LIDAR_SOURCE=serial reads a real STL-27L; the default is the CARLA UDP feed.
    Stl27lReader stl27l;
    if (qEnvironmentVariable("DASH_LIDAR_SOURCE") == "serial") {
        stl27l.setMountYawDeg(qEnvironmentVariable("DASH_LIDAR_YAW", "0").toDouble());
        stl27l.setMirrored(qEnvironmentVariable("DASH_LIDAR_MIRRORED", "0") == "1");
        QObject::connect(&stl27l, &Stl27lReader::scanReady, &lidarFeed, &LidarFeed::setScan);
        stl27l.open(qEnvironmentVariable("DASH_LIDAR_DEVICE", "/dev/ttyUSB0"),
                    qEnvironmentVariable("DASH_LIDAR_BAUD", "921600").toInt());
    }

    DetectionModel detectionModel;
    detectionModel.setDemo(qEnvironmentVariable("DASH_BEV_DEMO", "0") == "1");
    engine.rootContext()->setContextProperty("detectionModel", &detectionModel);

    DetectionFeed detectionFeed(&detectionModel);
    detectionFeed.listen(qEnvironmentVariableIntValue("DASH_DETECTION_PORT") > 0
                             ? quint16(qEnvironmentVariableIntValue("DASH_DETECTION_PORT"))
                             : quint16(5602));

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
