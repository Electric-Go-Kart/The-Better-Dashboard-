# The Better Dashboard

A Qt 6 + QML dashboard application for a go-kart/e-vehicle UI, with a C++ backend that can ingest CAN data (or simulate it) and push live values into the interface.

## What this project is

This repository contains:

- A **Qt Quick/QML frontend** for a touchscreen-style dashboard.
- A **C++ backend** for receiving and processing motor telemetry.
- A **CAN pipeline** built on `Qt SerialBus` (`QCanBus`) with support for `socketcan`.
- A **fake-data mode** currently active in code for development without hardware.

The target UX appears to be an `800x480` embedded dashboard screen.

## Tech stack

- **Language:** C++ (Qt framework) + QML
- **Build system:** CMake
- **Qt modules used:**
  - `Qt6::Quick`
  - `Qt6::SerialBus`
  - `QtQuick.VirtualKeyboard` (in QML)
- **Project type:** Qt Quick executable with a QML module (`URI QMLCDash`)

## High-level architecture

At runtime, the main data path is:

1. `main.cpp` creates:
   - `CANController`
   - `DashboardController`
2. Signals from `CANController` are connected to slots in `DashboardController`.
3. `DashboardController` is exposed to QML via:
   - context property: `dashboardController`
4. QML UI components (`Speedometer`, `BatteryGauge`, `CurrentDraw`, etc.) subscribe to `dashboardController` signals via `Connections`.

### Data flow details

- `CANController` decodes CAN frames into motor values (RPM/current/voltage).
- Each motor value is passed through `MotorDataProcessor`, which computes:
  - instant power
  - cumulative energy usage
  - estimated state of charge (SOC)
- `DashboardController` republishes these values as UI-facing signals.
- QML widgets update their visuals from those signals.

## Repository structure

```text
.
├── Backend/
│   ├── cancontroller.h/.cpp          # CAN IO + decode + fake data generator
│   ├── dashboardcontroller.h/.cpp    # Bridge layer between backend data and QML
│   └── motordataprocessor.h/.cpp     # Power/energy/SOC computations
├── UI/
│   └── Dashboard/
│       ├── App.qml                   # Main QML window wrapper
│       ├── dashboard/
│       │   ├── Dashboard.ui.qml      # Main layout/composition
│       │   ├── Speedometer.ui.qml
│       │   ├── BatteryGauge.ui.qml
│       │   ├── CurrentDraw.ui.qml
│       │   ├── DirectionButton.ui.qml
│       │   ├── Park.ui.qml
│       │   ├── Lights.ui.qml
│       │   ├── Lock.ui.qml
│       │   ├── Settings.ui.qml
│       │   └── ShutDown.ui.qml
│       └── images/
│           └── CSU-Ram-Rev.png
├── main.cpp                          # App startup + signal wiring + QML load
├── Main.qml                          # Root window + stack view + virtual keyboard
├── CMakeLists.txt                    # Build and Qt module configuration
├── test.h/.cpp                       # Legacy data simulator utility class
├── dashboard*.h/.cpp, dashboard.qml,
│   speedometer.qml                   # Legacy/older UI path (not main path)
└── .gitignore
```

## Current UI behavior

The dashboard currently composes:

- Center: `Speedometer`
- Right side: direction + park + lights controls
- Left side: shutdown + settings + lock controls
- Side gauges:
  - Battery SOC (`BatteryGauge`)
  - Current draw (`CurrentDraw`)

### Live-bound values

QML elements are currently wired to **left motor** signals:

- `Speedometer.ui.qml` listens to `onLeftRpmChanged`
- `BatteryGauge.ui.qml` listens to `onLeftSocChanged`
- `CurrentDraw.ui.qml` listens to `onLeftCurrentChanged`

## Build and run

## Prerequisites

Install:

- CMake `>= 3.16`
- A C++ compiler (Clang or GCC)
- Qt `6.8+` with at least:
  - `Qt Quick`
  - `Qt SerialBus`
  - `Qt Virtual Keyboard` (QML module)

> Note: `CMakeLists.txt` uses `qt_standard_project_setup(REQUIRES 6.8)`.

## Build

From repository root:

```bash
cmake -S . -B build
cmake --build build
```

## Run

```bash
./build/appQMLCDash
```

On some generators/platforms, the executable may be under a configuration folder (for example `build/Debug/`).

## CAN integration and simulation mode

`Backend/CANController` supports real CAN via:

- backend plugin: `socketcan`
- default interface: `vcan0`

Method:

- `bool initialize(const QString &interfaceName = "vcan0");`

### Important current behavior

- `main.cpp` **creates** `CANController` but does **not call** `initialize(...)`.
- `CANController` constructor currently starts a timer-based fake data generator (`generateFakeCanData()`), so UI data can move without CAN hardware.
- Fake data currently emits left motor RPM/current/SOC only.

If you want real CAN input, wire `initialize(...)` in startup and ensure your CAN interface is available.

## Notes on QML UI files

Files under `UI/Dashboard/dashboard/*.ui.qml` are Qt Design Studio UI form files. They are intended to stay mostly declarative and are typically edited with Qt Design Studio.

## Legacy and in-progress code paths

There are two dashboard controller sets in the repo:

- `Backend/dashboardcontroller.*` (active path used by `main.cpp`)
- root-level `dashboardcontroller.*` (older/legacy path)

There are also root-level widget/QML files from an older approach (`dashboard.*`, `Speedometer.qml`, etc.) that are not the primary app path loaded by `main.cpp` + `Main.qml`.

## Known limitations / TODOs

Based on current source:

- Several UI control buttons (`Direction`, `Park`, `Lights`, `Lock`, etc.) still use local `fake*` state in QML instead of backend commands.
- `Speedometer.ui.qml` references `DashboardController.locked`, which is not currently provided by the backend controller used in `main.cpp`.
- CAN decoding assumes a specific payload layout (RPM/current/voltage); adjust for your actual VESC/CAN protocol.
- Fake data interval comment and value are inconsistent (`start(600)` with comment saying 200ms).
- Right-motor values are partially wired but not yet reflected in the visible gauges.

## Development tips

- Keep backend signal names stable when editing QML `Connections`.
- Prefer implementing backend control slots/signals for UI actions before removing fake QML state toggles.
- If building on Linux with case-sensitive paths, ensure include filename casing exactly matches real filenames.

## Troubleshooting

- **Qt package not found:** point CMake at your Qt install (`CMAKE_PREFIX_PATH`).
- **QML module errors:** ensure `QtQuick.VirtualKeyboard` is installed in your Qt distribution.
- **No live data:** expected in real-CAN mode unless `CANController::initialize(...)` is called and interface is up.
- **SocketCAN issues:** verify interface (`vcan0` or real CAN iface) exists and is active.

## License

No license file is currently present in this repository. Add one (for example MIT, Apache-2.0, or proprietary) if you plan to distribute this code.

