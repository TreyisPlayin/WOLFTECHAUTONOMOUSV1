package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Example OpMode: builds a hard-coded preset and runs it.
 */
@Autonomous(name="MyAuto", group="Auto")
public class AutonomousOpMode extends LinearOpMode {
    @Override
    public void runOpMode() {
        // 1) init hardware & subsystems
        HardwareConfig robot    = new HardwareConfig(hardwareMap);
        Odometry odometry       = new Odometry(robot);
        PathPlanner planner     = new PathPlanner(new ZoneMap());
        CameraSync camSync      = new CameraSync(
                robot,
                /*baseline=*/5.0,
                /*FOV=*/60.0,
                /*imgWidth=*/320,
                /*targetID=*/1);
        IntakeSystem intake     = new IntakeSystem(robot);
        AutonomousRunner runner = new AutonomousRunner(
                this, robot, planner,
                odometry, camSync, intake);

        // 2) build preset here
        PresetManager pm = new PresetManager("Example");
        // mark start zone
        pm.addZone(0,0,10,10, ZoneMap.START_ZONE);
        // checkpoints
        pm.addCheckpoint(5,5,0,"start");
        pm.addCheckpoint(50,50,90,"pickup");
        pm.addCheckpoint(80,80,180,"score");

        waitForStart();

        // 3) run it
        runner.runPreset(pm);

        // 4) end
    }
}
