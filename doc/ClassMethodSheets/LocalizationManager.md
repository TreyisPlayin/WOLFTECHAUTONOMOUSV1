# LocalizationManager method quick-reference

Purpose: combine goBILDA Pinpoint odometry with AprilTag detections via VisionPortal. Use this when tele-op needs live pose and heading updates.

## Public API
- `LocalizationManager(HardwareMap hardwareMap, GoBildaPinpointDriver pinpoint)`: store hardware map and Pinpoint reference; constructs AprilTag processor but does **not** open the camera until `start()`.
- `void start()`: reset Pinpoint pose/IMU, ensure the vision portal is created/resumed, and zero the internal timer. Call once during init.
- `void update()`: keep the VisionPortal active, refresh pose from Pinpoint (`x_mm`, `y_mm`, `headingRad`), and override with the first AprilTag detection if available. Call every loop.
- `double getHeadingDeg()`: return the latest heading in degrees derived from `headingRad`.
- `boolean visionUpdateAvailable()`: true when at least one AprilTag detection was seen during the last `update()` call.

## Notes for tele-op integration
- Access the continuously updated pose via the public fields `x_mm`, `y_mm`, and `headingRad` after calling `update()` each loop.
- Vision corrections currently use a placeholder transform; adjust tag/robot offsets in code when field calibration is ready.
- Camera activation is lazy: `update()` automatically wakes the portal if it was paused/stopped.
