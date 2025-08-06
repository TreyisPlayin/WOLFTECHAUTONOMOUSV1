package org.firstinspires.ftc.teamcode;

/**
 * ActionHandler defines methods corresponding to checkpoint actions.
 * When a checkpoint’s action label matches one of these, the method is invoked.
 */
public class ActionHandler {

    /**
     * Called at a "pickup" checkpoint: aligns with target and intakes.
     */
    public static void lookForPickup(HardwareConfig robot,
                                     CameraSync cameraSync,
                                     IntakeSystem intake,
                                     Odometry odometry) {
        // 1. Align to object using dual cameras
        cameraSync.alignToTarget(/*tolerancePixels=*/20, /*toleranceInches=*/6.0);

        // 2. Activate intake to grab
        intake.activateIntake();

        // 3. Optionally wait for a brief moment to ensure grab
        try { Thread.sleep(300); } catch (InterruptedException ignored) {}

        // 4. Stop intake
        robot.intakeMotor.setPower(0);
    }

    /**
     * Called at a "score" checkpoint: moves forward slightly and releases.
     */
    public static void lookForScore(HardwareConfig robot,
                                    IntakeSystem intake) {
        // 1. Move forward a bit to ensure scoring position
        robot.leftFront.setPower(0.2);
        robot.rightFront.setPower(0.2);
        robot.leftBack.setPower(0.2);
        robot.rightBack.setPower(0.2);
        try { Thread.sleep(300); } catch (InterruptedException ignored) {}
        // stop
        robot.leftFront.setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        // 2. Release
        intake.release();
    }

    /**
     * Called at a "wait:XXX" checkpoint: waits for XXX milliseconds.
     */
    public static void waitAction(int ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }

    // Add more action methods as needed...
}
