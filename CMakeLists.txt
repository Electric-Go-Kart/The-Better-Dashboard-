cmake_minimum_required(VERSION 3.16)

project(QMLCDash VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Qt6 REQUIRED COMPONENTS Quick SerialBus)

qt_standard_project_setup(REQUIRES 6.8)

qt_add_executable(appQMLCDash
    main.cpp
)

qt_add_qml_module(appQMLCDash
    URI QMLCDash
    VERSION 1.0
    QML_FILES
    Main.qml
        #SOURCES dashboardcontroller.h dashboardcontroller.cpp
        SOURCES test.h test.cpp
        SOURCES Backend/cancontroller.h Backend/cancontroller.cpp
        SOURCES Backend/dashboardcontroller.h Backend/dashboardcontroller.cpp
        SOURCES Backend/motordataprocessor.h Backend/motordataprocessor.cpp

        QML_FILES
        QML_FILES UI/Dashboard/App.qml
        QML_FILES UI/Dashboard/dashboard/BatteryGauge.ui.qml
        QML_FILES UI/Dashboard/dashboard/CurrentDraw.ui.qml
        QML_FILES UI/Dashboard/dashboard/Dashboard.ui.qml
        QML_FILES UI/Dashboard/dashboard/DirectionButton.ui.qml
        QML_FILES UI/Dashboard/dashboard/Lights.ui.qml
        QML_FILES UI/Dashboard/dashboard/Lock.ui.qml
        QML_FILES UI/Dashboard/dashboard/Park.ui.qml
        QML_FILES UI/Dashboard/dashboard/Settings.ui.qml
        QML_FILES UI/Dashboard/dashboard/ShutDown.ui.qml
        QML_FILES UI/Dashboard/dashboard/Speedometer.ui.qml

        RESOURCES UI/Dashboard/images/CSU-Ram-Rev.png

        #QML_FILES Speedometer.qml
        #QML_FILES Dashboard.qml
)

# Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# If you are developing for iOS or macOS you should consider setting an
# explicit, fixed bundle identifier manually though.
set_target_properties(appQMLCDash PROPERTIES
#    MACOSX_BUNDLE_GUI_IDENTIFIER com.example.appQMLCDash
    MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
    MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
    MACOSX_BUNDLE TRUE
    WIN32_EXECUTABLE TRUE
)

target_link_libraries(appQMLCDash
    PRIVATE Qt6::Quick Qt${QT_VERSION_MAJOR}::SerialBus
)

include(GNUInstallDirs)
install(TARGETS appQMLCDash
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
