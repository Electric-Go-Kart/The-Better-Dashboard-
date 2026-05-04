#include "batterypersistence.h"

#include <QDebug>
#include <QDir>
#include <QFileInfo>
#include <QSettings>
#include <QStandardPaths>

namespace {
QString batteryIniPath()
{
    const QString base = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (base.isEmpty()) {
        return {};
    }
    QDir dir(base);
    if (!dir.mkpath(QStringLiteral("QMLCDash"))) {
        qWarning() << "BatteryPersistence: could not create config subdirectory under" << base;
        return {};
    }
    return dir.filePath(QStringLiteral("QMLCDash/battery_soc.ini"));
}
} // namespace

bool BatteryPersistence::load(float *leftWh, float *rightWh)
{
    if (!leftWh || !rightWh) {
        return false;
    }
    *leftWh = 0.0f;
    *rightWh = 0.0f;

    const QString path = batteryIniPath();
    if (path.isEmpty() || !QFileInfo::exists(path)) {
        return false;
    }

    QSettings settings(path, QSettings::IniFormat);
    settings.beginGroup(QStringLiteral("Battery"));
    const int schema = settings.value(QStringLiteral("schemaVersion"), 0).toInt();
    if (schema != kSchemaVersion && schema != 0) {
        qWarning() << "BatteryPersistence: unknown schemaVersion" << schema << "- ignoring file";
        settings.endGroup();
        return false;
    }
    *leftWh = static_cast<float>(settings.value(QStringLiteral("energyUsedWhLeft"), 0.0).toDouble());
    *rightWh = static_cast<float>(settings.value(QStringLiteral("energyUsedWhRight"), 0.0).toDouble());
    settings.endGroup();
    return true;
}

void BatteryPersistence::save(float leftWh, float rightWh)
{
    const QString path = batteryIniPath();
    if (path.isEmpty()) {
        qWarning() << "BatteryPersistence: no writable AppConfigLocation; skipping save";
        return;
    }

    QSettings settings(path, QSettings::IniFormat);
    settings.beginGroup(QStringLiteral("Battery"));
    settings.setValue(QStringLiteral("schemaVersion"), kSchemaVersion);
    settings.setValue(QStringLiteral("energyUsedWhLeft"), static_cast<double>(leftWh));
    settings.setValue(QStringLiteral("energyUsedWhRight"), static_cast<double>(rightWh));
    settings.endGroup();
    settings.sync();
    if (settings.status() != QSettings::NoError) {
        qWarning() << "BatteryPersistence: QSettings sync failed" << settings.status();
    }
}
