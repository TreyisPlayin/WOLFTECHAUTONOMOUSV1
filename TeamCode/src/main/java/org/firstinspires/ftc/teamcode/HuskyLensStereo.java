package org.firstinspires.ftc.teamcode;

/**
 * Dual HuskyLens alignment helper:
 * - Confirms both cameras see the same target ID
 * - Computes distance using your stereo formula (angles in degrees)
 * - Provides a simple center/approach routine (you can swap in VisionPortal webcam logic too)
 *
 * FTC HuskyLens overview & sample: ftc-docs HuskyLens page. :contentReference[oaicite:6]{index=6}
 */
public class HuskyLensStereo {
    private final HardwareConfig robot;
    private final int targetID;
    private final double BASELINE_A_IN; // inches between HuskyLens cameras
    private final double FOV_W_DEG;     // field-of-view of each HuskyLens (deg)
    private final int    IMG_W_PX;      // pixel width

    public HuskyLensStereo(HardwareConfig hw, int targetID, double baselineA, double fovWdeg, int imgWidth) {
        this.robot = hw;
        this.targetID = targetID;
        this.BASELINE_A_IN = baselineA;
        this.FOV_W_DEG = fovWdeg;
        this.IMG_W_PX  = imgWidth;
        // TODO: initialize both HuskyLens I2C/UART clients here
    }

    /** Replace with real HuskyLens calls. Returns true when both see the same ID. */
    public boolean bothSeeTarget() {
        return (getLeftID() == targetID) && (getRightID() == targetID);
    }

    // ===== stereo distance (angles in *degrees* as you required) =====
    public double calculateDistance(int Lpx, int Rpx) {
        double a1 = (Lpx*FOV_W_DEG/IMG_W_PX) + ((180.0 - FOV_W_DEG)/2.0);
        double a2 = (Rpx*FOV_W_DEG/IMG_W_PX) + ((180.0 - FOV_W_DEG)/2.0);
        double denomAng = 180.0 - (Lpx*FOV_W_DEG/IMG_W_PX + Rpx*FOV_W_DEG/IMG_W_PX) - (180.0 - FOV_W_DEG);
        double num = BASELINE_A_IN * Math.sin(Math.toRadians(a1)) * Math.sin(Math.toRadians(a2));
        double den = Math.sin(Math.toRadians(denomAng));
        return Math.abs(den) < 1e-6 ? -1 : num / den;
    }

    /** Center + approach using pixel center average and stereo distance. */
    public void alignToTarget(int pixelTolerance, double targetDistanceIn) {
        if (!bothSeeTarget()) return;

        int centerL = getLeftCenterPx();
        int centerR = getRightCenterPx();
        int avg = (centerL + centerR)/2;
        int offset = avg - (IMG_W_PX/2);

        // simple strafe if off-center (tune power!)
        double strafePower = 0.18 * Math.signum(offset);
        if (Math.abs(offset) > pixelTolerance) {
            // mecanum-like strafe
            robot.leftFront .setPower( strafePower);
            robot.rightFront.setPower(-strafePower);
            robot.leftBack  .setPower(-strafePower);
            robot.rightBack .setPower( strafePower);
        } else {
            stopDrive();
        }

        // Approach to target distance
        int Loff = getLeftOffsetPx();
        int Roff = getRightOffsetPx();
        double dist = calculateDistance(Loff, Roff);
        if (dist > 0) {
            double error = dist - targetDistanceIn;
            if (Math.abs(error) > 0.5) {
                double fwd = 0.20 * Math.signum(error);
                robot.leftFront .setPower( fwd);
                robot.rightFront.setPower( fwd);
                robot.leftBack  .setPower( fwd);
                robot.rightBack .setPower( fwd);
            } else {
                stopDrive();
            }
        }
    }

    private void stopDrive() {
        robot.leftFront .setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack  .setPower(0);
        robot.rightBack .setPower(0);
    }

    // ===== TODO: wire these to actual HuskyLens APIs =====
    private int getLeftID()        { return targetID; }
    private int getRightID()       { return targetID; }
    private int getLeftCenterPx()  { return IMG_W_PX/2; }
    private int getRightCenterPx() { return IMG_W_PX/2; }
    private int getLeftOffsetPx()  { return IMG_W_PX/4; }
    private int getRightOffsetPx() { return IMG_W_PX/4; }
}
