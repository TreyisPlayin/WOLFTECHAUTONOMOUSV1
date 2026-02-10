# FieldNav method quick-reference

Purpose: follow field-space paths using Pinpoint odometry with optional AprilTag drift correction and tag-seeking priority.

## Public API
- `FieldNav(HardwareMap hw, Telemetry tel, GoBildaPinpointDriver pin, String webcamName)`: start AprilTag vision (if webcam available), set default alliance (BLUE), seed pose from Pinpoint, and initialize curve follower.
- `void setAlliance(Alliance a)`: choose alliance so the correct goal tag ID is tracked (DECODE: BLUE=20, RED=24).
- `void driveTo(double targetX, double targetY)`: command a move to a field point (inches) with path-tangent heading.
- `void driveTo(double targetX, double targetY, Double finalHeadingDeg)`: command a move plus optional end-heading hold; `finalHeadingDeg` null skips end-facing.
- `boolean update()`: main loop step—updates fused pose, manages tag seeking, applies drift correction when tags are known, follows the path, and returns true once finished (arrived/timeout/canceled).
- `DriveCommand getDriveCommand()`: latest robot-frame drive/strafe/rotate command in [-1..1] to feed your drivetrain.
- `TagDelta getGoalTagDelta()`: latest camera-frame delta to the alliance goal tag (inches) with validity flag.
- `void setTagPose(int id, double xIn, double yIn, double hDeg)`: inform the library of a known AprilTag field pose to improve drift correction and tag-seeking direction.
- `Pose2d getPose()`: current fused pose (inches + heading deg) exposed via small immutable struct.

## Notes for tele-op integration
- Call `update()` every loop, then read `getDriveCommand()` to drive automatically, or blend with manual inputs.
- Use `setAlliance` during init and `setTagPose` for any field tags you care about so drift correction can engage.
- Tag seeking rotates in place after long gaps without goal detections; adjust `tagSeekAfterSec`/`seekRotatePower` constants inside the class if needed.
