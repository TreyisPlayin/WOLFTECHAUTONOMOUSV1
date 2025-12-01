# showcaseOp method quick-reference

Purpose: demonstration TeleOp showing three drive modes (rover tank, robot-centric mecanum, field-centric mecanum) using Pinpoint heading.

## Public API
- `void runOpMode()`: LinearOpMode entry that initializes hardware, reads Pinpoint each loop, allows driver to toggle drive mode with gamepad1 buttons (A/B/X), and mixes joystick inputs into wheel powers based on the selected mode.

## Notes for tele-op integration
- Uses `HardwareConfig` for drivetrain motors and `GoBildaPinpointDriver` heading when in field-centric mode.
- Adjust `maxSpeed` to cap overall drive power; leverages in-loop normalization to prevent saturation.
- The settings menu is entered with `start`; buttons A/B/X pick modes and exit the menu.
