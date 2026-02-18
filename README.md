# The Better Dashboard

A Qt 6 + QML dashboard application for a go-kart/e-vehicle UI, with a C++ backend that can ingest CAN data (or simulate it) and push live values into the interface.

## What this project is

This repository contains:

- A **Qt Quick/QML frontend** for a touchscreen-style dashboard.
- A **C++ backend** for receiving and processing motor telemetry.
- A **CAN pipeline** built on `Qt SerialBus` (`QCanBus`) with support for `socketcan`.
- A **single-motor** telemetry path designed for real CAN input on `can0`.

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
│   ├── cancontroller.h/.cpp          # CAN IO + VESC decode (single motor)
│   ├── dashboardcontroller.h/.cpp    # Bridge layer between backend data and QML
│   └── motordataprocessor.h/.cpp     # Power/energy/SOC computations
├── tools/
│   └── vesc_can_sim.cpp              # Standalone CAN simulator for frontend testing
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
├── Main.qml                          # Root window + dashboard + virtual keyboard
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

QML elements are wired to single-motor signals:

- `Speedometer.ui.qml` listens to `onRpmChanged`
- `BatteryGauge.ui.qml` listens to `onSocChanged`
- `CurrentDraw.ui.qml` listens to `onCurrentChanged`

## Build and run

## Prerequisites

Install:

- CMake `>= 3.16`
- A C++ compiler (Clang or GCC)
- Qt `6.2+` with at least:
  - `Qt Quick`
  - `Qt SerialBus`
  - `Qt Virtual Keyboard` (QML module)

> Note: `CMakeLists.txt` uses `qt_standard_project_setup(REQUIRES 6.2)`.

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

## CAN integration

`Backend/CANController` supports real CAN via:

- backend plugin: `socketcan`
- default interface: `can0`

Method:

- `bool initialize(const QString &interfaceName = "can0");`

### Important current behavior

- `main.cpp` initializes CAN at startup (`DASH_CAN_IFACE`, default `can0`).
- The controller ID used for VESC status filtering is configurable with `DASH_VESC_CONTROLLER_ID` (default `53`).
- VESC Status `1-4` frames are supported directly for single-motor telemetry.
- SOC handling prefers explicit VESC/BMS SOC packets when present; otherwise it can derive SOC from VESC Ah counters if battery capacity is configured.

Environment variables:

- `DASH_CAN_IFACE` (default: `can0`)
- `DASH_VESC_CONTROLLER_ID` (single motor CAN ID, default `53`)
- `DASH_BATTERY_CAPACITY_AH` (default `20`; enables SOC fallback from Status-2 Ah counters)

Current battery configuration baseline:

- 4x LiFePO4 modules in series
- 12.8 V each nominal -> 51.2 V pack nominal
- 20 Ah pack capacity
- ~1024 Wh nominal total energy

## Raspberry Pi OS + Boot2Qt notes

This project is configured for a single CAN interface (`can0`) and is compatible with:

- Raspberry Pi OS (development and bring-up)
- Boot2Qt on Raspberry Pi (deployment)

For MCP2515-based CAN SPI Click wiring (INT on GPIO6, CS on GPIO8/SPI0 CE0), enable SPI and overlay:

```ini
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=10000000,interrupt=6,spimaxfrequency=500000
```

Then bring up CAN:

```bash
sudo ip link set can0 up type can bitrate 500000
candump can0
```

Run app against hardware CAN:

```bash
DASH_CAN_IFACE=can0 DASH_VESC_CONTROLLER_ID=53 ./build/appQMLCDash
```

## Standalone CAN simulator for frontend testing

This repo includes a separate test program (`vescCanSim`) that publishes VESC-style Status `1-4` frames on SocketCAN.

Build:

```bash
cmake -S . -B build
cmake --build build
```

Run simulator:

```bash
SIM_CAN_IFACE=can0 SIM_VESC_CONTROLLER_ID=53 ./build/vescCanSim
```

Run dashboard in another terminal:

```bash
DASH_CAN_IFACE=can0 DASH_VESC_CONTROLLER_ID=53 ./build/appQMLCDash
```

Optional simulator tuning:

- `SIM_PERIOD_MS` (default `50`)
- `SIM_NOMINAL_VOLTAGE` (default `51.2`)
- `SIM_BATTERY_CAPACITY_AH` (default `20`)
- `SIM_SEND_EXPLICIT_SOC` (`1` default; sends VESC/BMS SOC packet `38`)

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
- CAN decoding assumes a specific payload layout (RPM/current/voltage); adjust for your actual VESC/CAN protocol.

## Development tips

- Keep backend signal names stable when editing QML `Connections`.
- Prefer implementing backend control slots/signals for UI actions before removing fake QML state toggles.
- If building on Linux with case-sensitive paths, ensure include filename casing exactly matches real filenames.

## Troubleshooting

- **Qt package not found:** point CMake at your Qt install (`CMAKE_PREFIX_PATH`).
- **QML module errors:** ensure `QtQuick.VirtualKeyboard` is installed in your Qt distribution.
- **No live data:** verify `candump can0` has frames and that `DASH_VESC_CONTROLLER_ID` matches your VESC ID.
- **SocketCAN issues:** verify interface (`can0`) exists and is active.

## License

No license file is currently present in this repository. Add one (for example MIT, Apache-2.0, or proprietary) if you plan to distribute this code.

