package org.firstinspires.ftc.teamcode;

/**
 * Defines behavior methods invoked at checkpoints or on entering zones.
 */
public class ActionHandler {

    /** Called at "start" checkpoint: can initialize sensors, delay, etc. */
    public static void doStart() {
        // e.g. calibrate IMU, wait for signal, etc.
    }

    /** Called at "pickup" checkpoint: align and intake. */
    public static void lookForPickup(HardwareConfig robot,
                                     CameraSync cameraSync,
                                     IntakeSystem intake,
                                     Odometry odo) {
        cameraSync.alignToTarget(20, 6.0);
        intake.activateIntake();
        try { Thread.sleep(300); } catch (InterruptedException ignored) {}
        robot.intakeMotor.setPower(0);
    }

    /** Called at "score": nudge and release. */
    public static void lookForScore(HardwareConfig robot,
                                    IntakeSystem intake) {
        robot.leftFront .setPower(0.2);
        robot.rightFront.setPower(0.2);
        robot.leftBack  .setPower(0.2);
        robot.rightBack .setPower(0.2);
        try { Thread.sleep(300); } catch (InterruptedException ignored) {}
        robot.leftFront .setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack  .setPower(0);
        robot.rightBack .setPower(0);
        intake.release();
    }

    /** Wait for specified milliseconds. */
    public static void waitAction(int ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }

    /** Stop/pause for X seconds at a "stop:X" action. */
    public static void stopForSeconds(int secs) {
        try { Thread.sleep(secs * 1000L); } catch (InterruptedException ignored) {}
    }

    /** Add more actions here as needed */
}
