package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.HashMap;
import java.util.Map;

/**
 * Register named actions and call them later from AutonomousRunner.
 *
 * Each action gets full access to:
 *  - HardwareConfig robot
 *  - HuskyLensStereo husky
 *  - VisionWebcamPortal portal (for AprilTags if you want)
 *  - IntakeSystem intake
 *  - Odometry odo (current pose)
 */
public class ActionRegistry {

    public interface Action {
        void run(HardwareConfig robot,
                 HuskyLensStereo husky,
                 VisionWebcamPortal portal,
                 IntakeSystem intake,
                 Odometry odo);
    }

    private final Map<String, Action> actions = new HashMap<>();

    public void register(String name, Action action) {
        actions.put(name.toLowerCase(), action);
    }

    public void run(String name,
                    HardwareConfig robot,
                    HuskyLensStereo husky,
                    VisionWebcamPortal portal,
                    IntakeSystem intake,
                    Odometry odo) {

        if (name == null) return;
        Action a = actions.get(name.toLowerCase());
        if (a != null) {
            a.run(robot, husky, portal, intake, odo);
        }
    }

    /** Default action set. */
    public static ActionRegistry defaults() {
        ActionRegistry r = new ActionRegistry();

        // === "start" – one-time things at first checkpoint ===
        r.register("start", (robot, husky, portal, intake, odo) -> {
            // Ensure drivetrain stopped & odometry updated once.
            robot.leftFront .setPower(0);
            robot.leftBack  .setPower(0);
            robot.rightFront.setPower(0);
            robot.rightBack .setPower(0);
            odo.update();
        });

        // === "pickup" – HuskyLens align + servo push into intake/catapult ===
        r.register("pickup", (robot, husky, portal, intake, odo) -> {
            // 1) Align to artifact using stereo cameras
            // pixelTolerance and distance are TUNEABLE.
            husky.alignToTarget(/*pixelTolerance*/20, /*targetDistanceIn*/6.0);

            // 2) Spin intake and/or push servo to shove artifact fully in
            // Option A: use intake motor
            robot.intakeMotor.setPower(1.0);
            try { Thread.sleep(350); } catch (InterruptedException ignored) {}
            robot.intakeMotor.setPower(0.0);

            // Option B (if you wired a specific pushServo or reuse intakeArm)
            Servo pusher = (robot.pushServo != null)
                    ? robot.pushServo
                    : robot.intakeArm; // fallback

            if (pusher != null) {
                try {
                    pusher.setPosition(1.0);   // extend
                    Thread.sleep(300);
                    pusher.setPosition(0.0);   // retract
                    Thread.sleep(200);
                } catch (InterruptedException ignored) {}
            }
        });

        // === "score" – auto-align to far goal, fire LEFT catapult, reload ===
        r.register("score", (robot, husky, portal, intake, odo) -> {
            // 1) Auto-align to far goal using odometry
            NavUtils.autoAlignToGoal(
                    robot,
                    odo,
                    NavUtils.FAR_GOAL_X_IN,
                    NavUtils.FAR_GOAL_Y_IN,
                    /*kP*/0.01,
                    /*headingTolDeg*/1.0,
                    /*timeoutMs*/1500
            );

            // 2) Fire LEFT catapult one time.
            // Requires: HardwareConfig.leftCatapult already mapped.
            if (robot.leftCatapult != null) {
                try {
                    robot.leftCatapult.setPower(1.0);
                    Thread.sleep(350);             // time to launch purple
                } catch (InterruptedException ignored) {
                } finally {
                    robot.leftCatapult.setPower(0.0);
                }
            }

            // 3) Reload catapult with loader servo
            // Requires: HardwareConfig.catapultLoader already mapped.
            if (robot.catapultLoader != null) {
                try {
                    robot.catapultLoader.setPosition(1.0); // push next purple in
                    Thread.sleep(300);
                    robot.catapultLoader.setPosition(0.0); // back to "ready"
                    Thread.sleep(200);
                } catch (InterruptedException ignored) {}
            }
        });

        // These two are still here so AutonomousRunner's "wait:NNN"/"stop:N" syntax works.
        r.register("wait", (robot, husky, portal, intake, odo) -> {
            // handled as a special string in AutonomousRunner.executeAction
        });
        r.register("stop", (robot, husky, portal, intake, odo) -> {
            // handled as a special string in AutonomousRunner.executeAction
        });

        return r;
    }
}
