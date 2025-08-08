package org.firstinspires.ftc.teamcode;

import java.util.HashMap;
import java.util.Map;

/** Register actions by name and call them later. */
public class ActionRegistry {
    public interface Action {
        void run(HardwareConfig robot, HuskyLensStereo husky, VisionWebcamPortal portal, IntakeSystem intake, Odometry odo);
    }

    private final Map<String,Action> actions = new HashMap<>();

    public void register(String name, Action action){ actions.put(name.toLowerCase(), action); }

    public void run(String name, HardwareConfig robot, HuskyLensStereo husky, VisionWebcamPortal portal, IntakeSystem intake, Odometry odo){
        Action a = actions.get(name.toLowerCase());
        if (a!=null) a.run(robot, husky, portal, intake, odo);
    }

    /** Defaults you can tweak. */
    public static ActionRegistry defaults(){
        ActionRegistry r=new ActionRegistry();

        r.register("start", (robot, husky, portal, intake, odo)-> {
            // IMU calibration, arm homing, etc. (add as needed)
        });

        r.register("pickup", (robot, husky, portal, intake, odo)-> {
            // Husky stereo align then intake
            husky.alignToTarget(/*pixelTol*/20, /*targetDist*/6.0);
            intake.activateIntake();
            try { Thread.sleep(300); } catch (InterruptedException ignored) {}
            robot.intakeMotor.setPower(0);
        });

        r.register("score", (robot, husky, portal, intake, odo)-> {
            // gentle nudge then release
            for (DcMotor m : new DcMotor[]{robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack}) m.setPower(0.2);
            try { Thread.sleep(250); } catch (InterruptedException ignored) {}
            for (DcMotor m : new DcMotor[]{robot.leftFront, robot.rightFront, robot.leftBack, robot.rightBack}) m.setPower(0);
            intake.release();
        });

        // Example parameterized actions: "wait:500" / "stop:2"
        r.register("wait", (robot, husky, portal, intake, odo)-> { /* handled in runner by parsing */ });
        r.register("stop", (robot, husky, portal, intake, odo)-> { /* handled in runner by parsing */ });

        return r;
    }
}
