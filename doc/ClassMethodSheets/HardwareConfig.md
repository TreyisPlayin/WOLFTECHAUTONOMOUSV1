# HardwareConfig method quick-reference

Purpose: centralize hardware handles for drivetrain, intake, sensors, shooter, LEDs, and vision devices.

## Public API
- `HardwareConfig(LinearOpMode opmode)`: store the owning op mode so hardwareMap/telemetry remain available.
- `void init(HardwareMap hardwareMap, Telemetry telemetry)`: map all motors, servos, sensors, and LEDs using configured names; resets Pinpoint IMU and sets right-side drive to reverse. Call once during init.

## Notes for tele-op integration
- After constructing with your `LinearOpMode`, call `init()` in `runOpMode()` to populate fields such as `leftFront`, `rightFront`, `pinpoint`, `huskyLens`, servos, and LEDs.
- Motors/servos are left in basic safe states (drive at 0, feed motors idle, gates/pushers closed); managers like `ShooterManager` adjust behavior further.
