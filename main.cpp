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
