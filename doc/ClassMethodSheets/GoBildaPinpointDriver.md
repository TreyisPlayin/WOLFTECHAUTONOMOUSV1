# GoBildaPinpointDriver method quick-reference

Purpose: I2C driver for the goBILDA Pinpoint odometry computer. Exposes pose/velocity plus configuration hooks.

## Core lifecycle
- `GoBildaPinpointDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned)`: construct with Lynx I2C client and set default address.
- `void update() / void update(ReadData data)`: poll device registers (full update or heading-only) to refresh cached encoder, pose, and velocity values.

## Configuration
- `void setOffsets(double xOffset, double yOffset)`: set pod offsets in millimeters.
- `void setOffsets(double xOffset, double yOffset, DistanceUnit distanceUnit)`: same with custom units.
- `void recalibrateIMU()`: trigger IMU calibration.
- `void resetPosAndIMU()`: reset position and IMU fusion.
- `void setEncoderDirections(EncoderDirection xEncoder, EncoderDirection yEncoder)`: declare encoder polarity.
- `void setEncoderResolution(GoBildaOdometryPods pods)`: apply preset ticks-per-mm for goBILDA pods.
- `void setEncoderResolution(double ticks_per_mm) / setEncoderResolution(double ticks_per_unit, DistanceUnit distanceUnit)`: custom resolution.
- `void setYawScalar(double yawOffset)`: scale yaw output when wheelbase differs from default.
- `Pose2D setPosition(Pose2D pos)`: set pose directly (mm, rad) and return Pose2D.
- `void setPosX(double posX, DistanceUnit distanceUnit) / void setPosY(...)`: set individual position axes.
- `void setHeading(double heading, AngleUnit angleUnit)`: force heading value.

## Device info
- `int getDeviceID()`, `int getDeviceVersion()`, `float getYawScalar()`, `DeviceStatus getDeviceStatus()`: hardware metadata.
- `int getLoopTime() / double getFrequency()`: last update loop time in ms and derived Hz.

## Pose & velocity getters
- `int getEncoderX() / getEncoderY()`: raw encoder ticks.
- `double getPosX() / getPosX(DistanceUnit distanceUnit)`: cached X position.
- `double getPosY() / getPosY(DistanceUnit distanceUnit)`: cached Y position.
- `double getHeading() / getHeading(AngleUnit) / getHeading(UnnormalizedAngleUnit)`: heading with unit options.
- `double getVelX() / getVelX(DistanceUnit distanceUnit)`, `double getVelY() / getVelY(DistanceUnit distanceUnit)`: linear velocities.
- `double getHeadingVelocity() / getHeadingVelocity(UnnormalizedAngleUnit)`: angular velocity.
- `float getXOffset(DistanceUnit) / float getYOffset(DistanceUnit)`: configured offsets.
- `Pose2D getPosition() / Pose2D getVelocity()`: convenience structs for pose and velocity.

## Notes for tele-op integration
- Call `update()` each loop before reading pose/velocity to keep cached values current.
- Use `resetPosAndIMU()` during init when pairing with `LocalizationManager` or `FieldNav` to zero odometry.
- If your encoder wiring flips directions, adjust `setEncoderDirections` once at startup.
