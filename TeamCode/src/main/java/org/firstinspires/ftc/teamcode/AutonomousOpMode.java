package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Runs your map-planned checkpoints with:
 *  - Odometry pose tracking
 *  - AprilTag fusion for exact field pose (webcam)
 *  - Dual HuskyLens stereo for pickup alignment
 *  - A* with smoothing & no-go avoidance
 *  - Dynamic obstacle updates from 4 distance sensors
 *  - Endgame "bolt" to final endpoint
 */
@Autonomous(name="TagFusedMapPlannerAuto", group="Auto")
public class AutonomousOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        // Hardware + subsystems
        HardwareConfig robot = new HardwareConfig(hardwareMap);
        Odometry odo         = new Odometry(robot);
        PresetManager pm     = new PresetManager();
        ZoneMap map          = pm.getZoneMap();
        PathPlanner planner  = new PathPlanner(map);
        HuskyLensStereo husky = new HuskyLensStereo(robot, /*targetID*/1, /*A*/5.0, /*FOV*/60.0, /*H*/320);
        VisionWebcamPortal portal = new VisionWebcamPortal(robot);
        AprilTagLocalizer tagLoc = new AprilTagLocalizer(
                portal.april,
                new AprilTagLocalizer.CamToRobot(
                        /*cam X offset from robot center*/ 5.0,
                        /*cam Y offset*/                    0.0,
                        /*cam yaw offset*/                  0.0
                )
        );
        // TODO: Replace these with the real field tag coordinates (inches + degrees)
        tagLoc.addFieldTag(1,  12, 132, 180);
        tagLoc.addFieldTag(2, 132, 132, -90);
        tagLoc.addFieldTag(3, 132,  12,   0);
        tagLoc.addFieldTag(4,  12,  12,  90);

        IntakeSystem intake = new IntakeSystem(robot);
        DistanceObstacleManager obstacleMgr = new DistanceObstacleManager(robot, map);
        ActionRegistry actions = ActionRegistry.defaults();

        // Mark no-go lines/zones on the map (example: field center line)
        map.markZone(72, 0, 72, 143, ZoneMap.FORBIDDEN);

        // Build your checkpoints (example)
        pm.addCheckpoint( 10,  10,   0, "start");
        pm.addCheckpoint( 40,  40,  90, "pickup");
        pm.addCheckpoint(100,  60, 180, "score");
        pm.addCheckpoint( 60, 100, 270, "pickup");
        pm.addCheckpoint(120, 120, 180, "score");

        // Endgame target to bolt to (e.g., park)
        Position finalBoltGoal = new Position(10, 135, 180); // replace with your park

        // Runner
        AutonomousRunner runner = new AutonomousRunner(
                this, robot, planner, odo, husky, portal, tagLoc, intake, obstacleMgr, actions,
                /*reserveMsForBolt*/ 4000, finalBoltGoal);

        telemetry.addLine("INIT COMPLETE - Waiting for START");
        telemetry.update();

        // === IMPORTANT: standard FTC start ===
        waitForStart();                     // ← will not run until Play is pressed
        if (!opModeIsActive()) return;      // safety

        // Run the plan
        runner.runPreset(pm);

        // end
    }
}
