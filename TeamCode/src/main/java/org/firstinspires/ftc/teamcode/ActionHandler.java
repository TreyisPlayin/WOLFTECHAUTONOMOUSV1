package org.firstinspires.ftc.teamcode;

/**
 * Stub methods invoked at checkpoints:
 * - start: initialization
 * - pickup: align+intake
 * - score: nudge+release
 * - wait:ms / stop:s
 */
public class ActionHandler {

    public static void doStart() {
        // e.g. calibrate IMU
    }

    public static void lookForPickup(HardwareConfig robot,
                                     CameraSync cam,
                                     IntakeSystem intake,
                                     Odometry odo) {
        cam.alignToTarget(20, 6.0);
        intake.activateIntake();
        try { Thread.sleep(300); } catch (InterruptedException ignored) {}
        robot.intakeMotor.setPower(0);
    }

    public static void lookForScore(HardwareConfig robot,
                                    IntakeSystem intake) {
        // drive forward a bit
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

    public static void waitAction(int ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }

    public static void stopForSeconds(int secs) {
        try { Thread.sleep(secs * 1000L); } catch (InterruptedException ignored) {}
    }
}
