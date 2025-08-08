package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;

/**
 * Executes: for each checkpoint → plan (A* + smoothing), Pure Pursuit follow, then action.
 * Dynamic obstacle updates; re-plan if blocked. AprilTag fusion runs continuously.
 */
public class AutonomousRunner {
    private final LinearOpMode op;
    private final HardwareConfig robot;
    private final PathPlanner planner;
    private final Odometry odo;
    private final HuskyLensStereo husky;
    private final VisionWebcamPortal portal;
    private final AprilTagLocalizer tagLoc;
    private final IntakeSystem intake;
    private final DistanceObstacleManager obstacles;
    private final ActionRegistry actions;

    private final long reserveMsForBolt; // time reserve for endgame bolt
    private final Position finalBoltGoal;

    // Heading PID still used for quick rotates (e.g., before action)
    private final PIDController rotPID = new PIDController(0.010, 0.0, 0.0005);
    private static final double HEADING_TOL = 1.0; // degrees

    // Pure Pursuit tuners (start here)
    private double LOOKAHEAD_IN = 12.0;
    private double MAX_XY_POWER = 0.7;
    private double MAX_TURN_PWR = 0.9;

    public AutonomousRunner(LinearOpMode op,
                            HardwareConfig robot,
                            PathPlanner planner,
                            Odometry odo,
                            HuskyLensStereo husky,
                            VisionWebcamPortal portal,
                            AprilTagLocalizer tagLoc,
                            IntakeSystem intake,
                            DistanceObstacleManager obstacles,
                            ActionRegistry actions,
                            long reserveMsForBolt,
                            Position finalBoltGoal) {
        this.op=op; this.robot=robot; this.planner=planner; this.odo=odo;
        this.husky=husky; this.portal=portal; this.tagLoc=tagLoc; this.intake=intake;
        this.obstacles=obstacles; this.actions=actions;
        this.reserveMsForBolt = reserveMsForBolt; this.finalBoltGoal = finalBoltGoal;
    }

    public void runPreset(PresetManager pm) {
        long startMs = System.currentTimeMillis();

        for (Checkpoint cp : pm.getCheckpoints()) {
            if (!op.opModeIsActive()) break;

            // 1) Path plan to this checkpoint (A* returns smoothed list already)
            Position cur = odo.getCurrentPosition();
            List<Position> smoothPath = planner.aStar(cur, new Position(cp.x, cp.y, cp.heading));

            // 2) Pure Pursuit follow (with tag fusion + obstacle updates + replanning)
            followPathPurePursuit(smoothPath, new Position(cp.x, cp.y, cp.heading));

            // 3) Optional: rotate to exact heading before action
            rotateTo(cp.heading);

            // 4) Execute named action
            executeAction(cp.action);

            // Reserve time for endgame "bolt"
            if (System.currentTimeMillis() - startMs > (op.getRuntime()*0 + 0) // runtime not reliable here
                    && (reserveMsForBolt > 0) && (timeLeftMs() < reserveMsForBolt)) {
                break;
            }
        }

        // Endgame bolt if configured
        if (finalBoltGoal != null && op.opModeIsActive()) {
            List<Position> boltPath = planner.aStar(odo.getCurrentPosition(), finalBoltGoal);
            followPathPurePursuit(boltPath, finalBoltGoal);
        }

        stopDrive();
    }

    private void followPathPurePursuit(List<Position> path, Position goal) {
        PurePursuitController pp = new PurePursuitController(robot, odo, LOOKAHEAD_IN, MAX_XY_POWER, MAX_TURN_PWR);
        pp.setPath(path);

        while (op.opModeIsActive() && !pp.isFinished()) {
            // 1) keep odometry fresh
            odo.update(); // your deadwheel update step :contentReference[oaicite:8]{index=8}

            // 2) dynamic obstacles => re-plan if needed
            obstacles.updateObstacles(odo.getCurrentPosition());

            // 3) AprilTag fusion (blended) each loop if a good tag is visible
            tagLoc.maybeFuseWithOdometry(odo); // blended, not snap anymore :contentReference[oaicite:9]{index=9}

            // 4) if path vanished/blocked, re-plan
            if (path == null || path.isEmpty()) {
                path = planner.aStar(odo.getCurrentPosition(), goal);
                pp.setPath(path);
                if (path.isEmpty()) { pp.stopDrive(); return; }
            }

            // 5) Advance along path
            PurePursuitController.Command cmd = pp.update();
            pp.applyDrive(cmd);

            // 6) Re-plan if a very near obstacle shows up (8" trip)
            double near = Math.min(Math.min(
                            robot.distFront.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH),
                            robot.distLeft .getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)),
                    Math.min(
                            robot.distRight.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH),
                            robot.distBack .getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
            if (near > 0 && near < 8) {
                pp.stopDrive();
                path = planner.aStar(odo.getCurrentPosition(), goal);
                pp.setPath(path);
            }

            op.telemetry.addData("Goal", goal);
            op.telemetry.addData("Pose", odo.getCurrentPosition());
            op.telemetry.update();
        }

        pp.stopDrive();
    }

    private void executeAction(String label) {
        String act = label.toLowerCase();
        if (act.startsWith("wait:")) {
            try { Thread.sleep(Integer.parseInt(act.substring(5))); } catch (InterruptedException ignored) {}
            return;
        }
        if (act.startsWith("stop:")) {
            try { Thread.sleep(1000L*Integer.parseInt(act.substring(5))); } catch (InterruptedException ignored) {}
            return;
        }
        actions.run(act, robot, husky, portal, intake, odo);
    }

    private void rotateTo(double targetDeg) {
        rotPID.reset();
        while (op.opModeIsActive()) {
            odo.update();
            double cur = odo.getCurrentPosition().getHeading();
            double err = angleDiff(targetDeg, cur);
            if (Math.abs(err) < HEADING_TOL) break;
            double p = clamp(rotPID.calculate(err), -0.5, 0.5);
            // tank turn in place
            robot.leftFront .setPower( p);
            robot.leftBack  .setPower( p);
            robot.rightFront.setPower(-p);
            robot.rightBack .setPower(-p);
            op.telemetry.addData("RotErr", err);
            op.telemetry.update();
        }
        stopDrive();
    }

    // Helpers
    private long timeLeftMs() { return Long.MAX_VALUE; } // placeholder if you later add a match timer
    private double angleDiff(double target, double current) {
        double d = ((target - current + 180) % 360) - 180;
        return (d < -180) ? d + 360 : d;
    }
    private void stopDrive() {
        robot.leftFront .setPower(0);
        robot.leftBack  .setPower(0);
        robot.rightFront.setPower(0);
        robot.rightBack .setPower(0);
    }
    private double clamp(double v,double lo,double hi){ return Math.max(lo, Math.min(hi, v)); }
}
