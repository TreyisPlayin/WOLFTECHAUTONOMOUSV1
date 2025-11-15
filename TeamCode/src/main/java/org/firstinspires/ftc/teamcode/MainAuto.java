package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Basic autonomous:
 *  - hardware.init()
 *  - uses FieldNav (Pinpoint + AprilTag) for localization and tag reading
 *  - reads distance to the alliance GOAL AprilTag
 *  - computes launch power from that distance (your formula)
 *  - fires both catapults with that power
 */
@Autonomous(name = "mainAuto", group = "Auto")
public class MainAuto extends LinearOpMode {

    private HardwareConfig robot;
    private FieldNav nav;
    private double lastShotRequiredRpm = 0.0;

    /**
     * Shooter model constants used by {@link #computeLaunchPowerFromXDistance(double)}.
     * Tune these for the real robot.
     */
    private static final double MAX_SHOOTER_RPM = 5600.0;      // Free speed of the shooter wheel
    private static final double LAUNCH_ANGLE_DEG = 50.0;       // Launch angle of the projectile relative to the floor
    private static final double SHOOTER_WHEEL_RADIUS_IN = 2.0; // Radius of the shooter wheel in inches
    private static final double GRAVITY_IN_PER_S2 = 386.09;    // g in inches/second^2

    @Override
    public void runOpMode() throws InterruptedException {
        // 1) Hardware init (uses your existing HardwareConfig)
        robot = new HardwareConfig(this);
        robot.init(hardwareMap, telemetry);

        if (robot.pinpoint == null) {
            telemetry.addLine("Pinpoint IMU not available. Autonomous cannot run.");
            telemetry.update();
            return;
        }

        // 2) FieldNav: Pinpoint + AprilTags via VisionPortal
        // NOTE: change "Webcam 1" if your RC config uses another name
        nav = new FieldNav(hardwareMap, telemetry, robot.pinpoint, "Webcam 1");

        // Set alliance so FieldNav picks the correct GOAL tag ID (BLUE=20, RED=24)
        // Change this depending on which side you're running.
        nav.setAlliance(FieldNav.Alliance.BLUE);

        telemetry.addLine("mainAuto init complete. Waiting for start...");
        telemetry.update();

        // Optional: show live pose & tag info during init loop
        while (!isStarted() && !isStopRequested()) {
            nav.update(); // updates Pinpoint + scans for tags

            FieldNav.Pose2d pose = nav.getPose();
            telemetry.addData("Pose X (in)", "%.1f", pose.x);
            telemetry.addData("Pose Y (in)", "%.1f", pose.y);
            telemetry.addData("Heading (deg)", "%.1f", pose.hDeg);

            FieldNav.TagDelta td = nav.getGoalTagDelta();
            if (td != null && td.valid) {
                telemetry.addData("Goal tag seen?", true);
                telemetry.addData("Tag cam X (in)", "%.1f", td.xIn);
                telemetry.addData("Tag cam Y (in)", "%.1f", td.yIn);
            } else {
                telemetry.addData("Goal tag seen?", false);
            }

            telemetry.update();
            sleep(20);
        }

        waitForStart();
        if (isStopRequested()) return;

        // 3) After start: give FieldNav a short window to pick up the goal tag
        FieldNav.TagDelta bestTag = null;
        double startTime = getRuntime();

        while (opModeIsActive() && (getRuntime() - startTime) < 1.5) {
            nav.update();
            FieldNav.TagDelta cur = nav.getGoalTagDelta();
            if (cur != null && cur.valid) {
                bestTag = cur;
            }

            FieldNav.Pose2d pose = nav.getPose();
            telemetry.addData("Pose X (in)", "%.1f", pose.x);
            telemetry.addData("Pose Y (in)", "%.1f", pose.y);
            telemetry.addData("Heading (deg)", "%.1f", pose.hDeg);

            if (cur != null) {
                telemetry.addData("Tag valid", cur.valid);
                telemetry.addData("Tag cam X (in)", "%.1f", cur.xIn);
                telemetry.addData("Tag cam Y (in)", "%.1f", cur.yIn);
            } else {
                telemetry.addLine("No tag detections yet...");
            }

            telemetry.update();
            sleep(20);
        }

        // 4) Choose what we call "X distance" for your formula
        double xDistanceIn;

        if (bestTag != null && bestTag.valid) {
            // bestTag.yIn = distance forward from camera to the tag (inches)
            // This is usually what you care about for shot power (distance AWAY from the goal).
            xDistanceIn = bestTag.yIn;
        } else {
            // Fallback distance if we never see the tag
            xDistanceIn = 36.0; // <- pick a reasonable default in inches
            telemetry.addLine("No valid goal tag; using default distance: " + xDistanceIn + " in");
        }

        // 5) Compute launch power from X distance (you plug your formula in here)
        double launchPower = computeLaunchPowerFromXDistance(xDistanceIn);
        double requiredRpm = Math.min(lastShotRequiredRpm, MAX_SHOOTER_RPM);

        telemetry.addData("X distance used (in)", "%.1f", xDistanceIn);
        telemetry.addData("Launch power", "%.2f", launchPower);
        telemetry.addData("Required RPM", "%.0f", requiredRpm);
        telemetry.addData("Launch angle (deg)", "%.1f", LAUNCH_ANGLE_DEG);
        telemetry.addData("Max shooter RPM", "%.0f", MAX_SHOOTER_RPM);
        telemetry.update();

        // 6) Fire both catapults with that power
        fireBothCatapults(launchPower, 800); // 800 ms = guess; tune for your mechanism

        telemetry.addLine("mainAuto complete.");
        telemetry.update();
        sleep(500);
    }

    /**
     * YOUR SHOT-FORMULA GOES HERE.
     *
     * xDistanceIn = forward distance from the camera to the goal AprilTag (inches).
     * Replace the body with your actual function P(x).
     */
    private double computeLaunchPowerFromXDistance(double xDistanceIn) {
        double angleRad = Math.toRadians(LAUNCH_ANGLE_DEG);
        double sinTwoTheta = Math.sin(2 * angleRad);

        if (sinTwoTheta <= 1e-6 || SHOOTER_WHEEL_RADIUS_IN <= 0.0 || MAX_SHOOTER_RPM <= 0.0) {
            lastShotRequiredRpm = 0.0;
            return 0.0;
        }

        // Projectile range equation solved for muzzle velocity when launch/landing heights are equal.
        double requiredVelocity = Math.sqrt(Math.max(0.0, xDistanceIn * GRAVITY_IN_PER_S2 / sinTwoTheta));

        // Convert the linear velocity to the wheel RPM required to provide that tangential speed.
        double requiredRpm = (requiredVelocity * 60.0) / (2.0 * Math.PI * SHOOTER_WHEEL_RADIUS_IN);
        lastShotRequiredRpm = requiredRpm;

        // Normalize against the maximum achievable shooter speed and clamp into motor power range.
        double power = requiredRpm / MAX_SHOOTER_RPM;
        return clamp(power, 0.0, 1.0);
    }

    /**
     * Basic time-based catapult fire. Both motors run at the same power.
     * Make sure leftCatapult/rightCatapult are mapped in HardwareConfig.
     */
    private void fireBothCatapults(double power, long durationMs) {
        DcMotor left  = robot.leftCatapult;
        DcMotor right = robot.rightCatapult;

        power = clamp(power, -1.0, 1.0);

        if (left != null) {
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left.setPower(power);
        }
        if (right != null) {
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setPower(power);
        }

        sleep(durationMs);

        if (left != null)  left.setPower(0.0);
        if (right != null) right.setPower(0.0);
    }

    private double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }
}
