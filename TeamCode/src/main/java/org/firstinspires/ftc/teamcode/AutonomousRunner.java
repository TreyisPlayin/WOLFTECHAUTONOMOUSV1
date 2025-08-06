package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;

/**
 * AutonomousRunner executes a preset’s checkpoints in order.
 * It rotates, drives to each (x,y), then calls the ActionHandler.
 */
public class AutonomousRunner {
    private final LinearOpMode opMode;
    private final HardwareConfig robot;
    private final PathPlanner planner;
    private final Odometry odometry;
    private final CameraSync cameraSync;
    private final IntakeSystem intake;

    public AutonomousRunner(LinearOpMode opMode,
                            HardwareConfig robot,
                            PathPlanner planner,
                            Odometry odometry,
                            CameraSync cameraSync,
                            IntakeSystem intake) {
        this.opMode      = opMode;
        this.robot       = robot;
        this.planner     = planner;
        this.odometry    = odometry;
        this.cameraSync  = cameraSync;
        this.intake      = intake;
    }

    public void runPreset(PresetManager preset) {
        List<Checkpoint> cps = preset.getCheckpoints();
        for (Checkpoint cp : cps) {
            if (!opMode.opModeIsActive()) break;

            // Display which checkpoint we’re at
            opMode.telemetry.addData("Checkpoint", cp.toString());
            opMode.telemetry.update();

            // 1) Rotate to desired heading
            rotateTo(cp.heading);

            // 2) Drive to (x,y)
            List<Position> path = planner.aStar(odometry.getCurrentPosition(),
                    new Position(cp.x, cp.y, cp.heading));
            planner.followPath(path, robot);

            // 3) Execute action
            String action = cp.action.toLowerCase();
            if (action.equals("pickup")) {
                ActionHandler.lookForPickup(robot, cameraSync, intake, odometry);
            } else if (action.equals("score")) {
                ActionHandler.lookForScore(robot, intake);
            } else if (action.startsWith("wait:")) {
                int ms = Integer.parseInt(action.substring(5));
                ActionHandler.waitAction(ms);
            }
        }
    }

    /** Rotate in place to the target heading (stub – implement with PID). */
    private void rotateTo(double targetHeading) {
        // TODO: implement rotation using odometry.getCurrentPosition().getHeading()
    }
}
