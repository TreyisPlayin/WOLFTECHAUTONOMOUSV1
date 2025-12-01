# LocalizationManager Guide

This guide summarizes what the current `LocalizationManager` implementation does and how to use it from tele-op code.

## Capabilities

- Maintains the robot pose using the goBILDA Pinpoint odometry driver and exposes the pose as `x_mm`, `y_mm`, and `headingRad` fields that update each loop.
- Provides a vision pipeline using a `VisionPortal` with an `AprilTagProcessor`. The portal is lazily created and automatically resumed if it becomes paused or stopped.
- Uses the latest AprilTag detection (if available) to override the odometry pose with the camera-derived position and heading, so vision can correct drift when a tag is visible.
- Offers helper utilities:
  - `start()` to reset odometry/IMU and activate the vision portal.
  - `update()` to refresh the odometry pose, keep the vision pipeline alive, and apply tag-based corrections.
  - `getHeadingDeg()` to retrieve heading in degrees.
  - `visionUpdateAvailable()` to check if a tag detection was available during the last update.

## Tele-op usage example

```java
LocalizationManager loc = new LocalizationManager(hardwareMap, pinpoint);

// In init
loc.start();

// In loop
loc.update();
telemetry.addData("X (mm)", loc.x_mm);
telemetry.addData("Y (mm)", loc.y_mm);
telemetry.addData("Heading (deg)", loc.getHeadingDeg());
if (loc.visionUpdateAvailable()) {
    telemetry.addLine("Vision correction applied");
}
telemetry.update();
```

### Notes
- Configure the camera name (`"Webcam 1"`) in the hardware map to match your configuration.
- The tag-to-field transform is currently a placeholder: the camera-derived pose is trusted directly. Customize the transform and tag locations when you need field-accurate corrections.
- Call `start()` during the init phase so the camera opens and odometry resets before the match starts.
- Ensure `update()` is invoked every loop to keep both odometry and vision data fresh.
