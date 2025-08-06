package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;

/**
 * Drives through the preset’s checkpoints in order:
 *  rotate → pathfind → execute action.
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
        this.opMode     = opMode;
        this.robot      = robot;
        this.planner    = planner;
        this.odometry   = odometry;
        this.cameraSync = cameraSync;
        this.intake     = intake;
    }

    /** Execute all checkpoints in the given preset. */
    public void runPreset(PresetManager preset) {
        List<Checkpoint> cps = preset.getCheckpoints();
        for (Checkpoint cp : cps) {
            if (!opMode.opModeIsActive()) break;

            opMode.telemetry.addData("Next", cp);
            opMode.telemetry.update();

            // 1) rotate to heading
            rotateTo(cp.heading);

            // 2) pathfind & follow
            List<Position> path = planner.aStar(
                    odometry.getCurrentPosition(),
                    new Position(cp.x, cp.y, cp.heading)
            );
            planner.followPath(path, robot);

            // 3) invoke action
            String act = cp.action.toLowerCase();
            if (act.equals("start")) {
                ActionHandler.doStart();
            } else if (act.equals("pickup")) {
                ActionHandler.lookForPickup(robot, cameraSync, intake, odometry);
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
    }

    /** Rotate to absolute target heading (degrees). Stub for PID control. */
    private void rotateTo(double targetDeg) {
        // TODO: implement using odometry.getCurrentPosition().getHeading()
    }
}
