# Boot2Qt CAN Deployment

This project now defaults to production CAN on `can0` and keeps `vcan0` for explicit bench testing.

## Runtime configuration

Set these environment variables before launching the app:

- `DASH_CAN_INTERFACE` (default: `can0`)
- `DASH_CAN_TEST_MODE` (`0` by default, set to `1` for `vcan0` loopback fake traffic)
- `DASH_LEFT_CONTROLLER_ID` (default: `1`)
- `DASH_RIGHT_CONTROLLER_ID` (default: `2`)
- `DASH_PARK_BRAKE_CURRENT_A` (default: `12.0`)

Example:

```sh
export DASH_CAN_INTERFACE=can0
export DASH_CAN_TEST_MODE=0
export DASH_LEFT_CONTROLLER_ID=1
export DASH_RIGHT_CONTROLLER_ID=2
export DASH_PARK_BRAKE_CURRENT_A=15
```

## Boot2Qt image prerequisites

1. Keep `Qt6::SerialBus` linked in CMake (already enabled in this repo).
2. Ensure the `socketcan` plugin is available on target:
   - `plugins/canbus/libqtsocketcanbus.so`
3. Ensure Linux CAN modules are available in image/kernel:
   - `can`, `can_raw`, and controller driver module.

## Bring up CAN at startup

Use the provided startup script and systemd unit:

- Script: `deploy/can0-setup.sh`
- Unit: `deploy/systemd/dash-can0.service`

Install on target:

```sh
sudo install -m 0755 deploy/can0-setup.sh /usr/local/bin/can0-setup.sh
sudo install -m 0644 deploy/systemd/dash-can0.service /etc/systemd/system/dash-can0.service
sudo systemctl daemon-reload
sudo systemctl enable --now dash-can0.service
```

## Qt Creator / Boot2Qt note

Do not rely on user-local `CMakeLists.txt.user` for deployment assumptions. Keep deployment behavior documented in this file and controlled by environment variables + systemd configuration on target.
