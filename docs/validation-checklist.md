# Validation Checklist

## Bench (`vcan0`) replay checks

- [ ] Launch with `DASH_CAN_INTERFACE=vcan0` and `DASH_CAN_TEST_MODE=1`.
- [ ] Confirm telemetry updates in UI:
  - [ ] RPM gauge updates.
  - [ ] Current bar updates.
  - [ ] SOC bar updates.
- [ ] Toggle controls and verify backend state changes:
  - [ ] Park toggles between `Park` and `Drive`.
  - [ ] Direction toggles between `Forward` and `Reverse` only when not parked.
  - [ ] Lights and Lock toggles persist in UI.
- [ ] Verify transition guard:
  - [ ] Direction/park changes are rejected when RPM is above safe threshold.

## Hardware-in-loop (`can0`) checks

- [ ] `can0` is up at expected bitrate before app starts.
- [ ] App starts with `DASH_CAN_TEST_MODE=0` and receives live telemetry.
- [ ] Park command engages drag brake current on both motor controllers.
- [ ] Park heartbeat keeps brake command alive while parked.
- [ ] Exiting Park clears brake current and sends neutral current command.
- [ ] Disconnect CAN physically and verify app enters faulted state.
- [ ] Reconnect CAN and verify app can recover after fault clear action.

## Safety checks

- [ ] No UI control uses local fake state toggles.
- [ ] Fault state blocks direction and park transitions until cleared.
- [ ] On CAN disconnect/error, drivetrain state enters `Fault`.
- [ ] Park is software-managed and never silently exits.
