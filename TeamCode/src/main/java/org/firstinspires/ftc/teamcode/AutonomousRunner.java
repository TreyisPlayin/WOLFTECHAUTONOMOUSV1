package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;

/**
 * Drives through the preset’s checkpoints in order:
 *   1) rotateTo(heading)
 *   2) driveTo(x, y)
 *   3) perform action
 */
public class AutonomousRunner {
    private final LinearOpMode opMode;
    private final HardwareConfig robot;
    private final PathPlanner planner;
    private final Odometry odometry;
    private final CameraSync camSync;
    private final IntakeSystem intake;

    // PID controllers and tolerances
    private final PIDController rotPID = new PIDController(0.01, 0.0, 0.0005);
    private final PIDController distPID = new PIDController(0.1, 0.0, 0.01);
    private static final double HEADING_TOLERANCE = 1.0;  // degrees
    private static final double DISTANCE_TOLERANCE = 0.5; // inches

    public AutonomousRunner(LinearOpMode opMode,
                            HardwareConfig robot,
                            PathPlanner planner,
                            Odometry odometry,
                            CameraSync camSync,
                            IntakeSystem intake) {
        this.opMode   = opMode;
        this.robot    = robot;
        this.planner  = planner;
        this.odometry = odometry;
        this.camSync  = camSync;
        this.intake   = intake;
    }

    /** Main entry: run through all checkpoints in the preset. */
    public void runPreset(PresetManager pm) {
        List<Checkpoint> cps = pm.getCheckpoints();
        for (Checkpoint cp : cps) {
            if (!opMode.opModeIsActive()) break;

            opMode.telemetry.addData("Next CP", cp);
            opMode.telemetry.update();

            // 1) Rotate to desired heading
            rotateTo(cp.heading);

            // 2) Drive to (x,y)
            driveTo(cp.x, cp.y);

            // 3) Perform the checkpoint's action
            executeAction(cp.action);
        }
    }

    /** Rotate robot in place to absolute heading (degrees). */
    private void rotateTo(double targetDeg) {
        rotPID.reset();
        while (opMode.opModeIsActive()) {
            odometry.update();
            double current = odometry.getCurrentPosition().getHeading();
            double error = shortestAngleDiff(targetDeg, current);
            if (Math.abs(error) < HEADING_TOLERANCE) break;

            double power = rotPID.calculate(error);
            power = clamp(power, -0.5, 0.5);

            // Tank turn
            robot.leftFront .setPower( power);
            robot.leftBack  .setPower( power);
            robot.rightFront.setPower(-power);
            robot.rightBack .setPower(-power);

            opMode.telemetry.addData("RotErr", "%.1f", error);
            opMode.telemetry.update();
        }
        stopDriveMotors();
    }

    /** Drive straight from current pose to (xTarget,yTarget). */
    private void driveTo(double xTarget, double yTarget) {
        distPID.reset();
        Position start = odometry.getCurrentPosition();
        double totalDist = start.distanceTo(new Position(xTarget, yTarget, 0));

        while (opMode.opModeIsActive()) {
            odometry.update();
            Position cur = odometry.getCurrentPosition();
            double remaining = cur.distanceTo(new Position(xTarget, yTarget, 0));
            double error = remaining;

            if (error < DISTANCE_TOLERANCE) break;

            double power = distPID.calculate(error);
            power = clamp(power, -0.6, 0.6);

            // Drive forward/back
            robot.leftFront .setPower( power);
            robot.leftBack  .setPower( power);
            robot.rightFront.setPower( power);
            robot.rightBack .setPower( power);

            opMode.telemetry.addData("DistRem", "%.1f/%.1f", remaining, totalDist);
            opMode.telemetry.update();
        }
        stopDriveMotors();
    }

    /** Helper to perform the action label from a checkpoint. */
    private void executeAction(String action) {
        String act = action.toLowerCase();
        if (act.equals("start")) {
            ActionHandler.doStart();
        } else if (act.equals("pickup")) {
            ActionHandler.lookForPickup(robot, camSync, intake, odometry);
        } else if (act.equals("score")) {
            ActionHandler.lookForScore(robot, intake);
        } else if (act.startsWith("wait:")) {
            int ms = Integer.parseInt(act.substring(5));
            ActionHandler.waitAction(ms);
        } else if (act.startsWith("stop:")) {
            int s = Integer.parseInt(act.substring(5));
            ActionHandler.stopForSeconds(s);
        }
    }

    /** Stop all four drive motors. */
    private void stopDriveMotors() {
        robot.leftFront .setPower(0);
        robot.leftBack  .setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack .setPower(0);
    }

    /** Normalize and get shortest difference between two headings. */
    private double shortestAngleDiff(double target, double current) {
        double diff = ((target - current + 180) % 360) - 180;
        return diff < -180 ? diff + 360 : diff;
    }

    /** Clamp value to [min, max]. */
    private double clamp(double v, double min, double max) {
        return v < min ? min : (v > max ? max : v);
    }
}
