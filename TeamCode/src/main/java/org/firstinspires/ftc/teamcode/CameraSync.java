package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Stereo alignment & distance using two HuskyLens cameras.
 * Replace stubs with your actual HuskyLens SDK calls.
 */
public class CameraSync {
    private final HardwareConfig robot;
    private final double A, W;  // baseline inches, FOV degrees
    private final int H;         // image width px
    private final int targetID;

    public CameraSync(HardwareConfig hw,
                      double baselineInches,
                      double fovDegrees,
                      int imageWidthPx,
                      int targetID) {
        this.robot    = hw;
        this.A        = baselineInches;
        this.W        = fovDegrees;
        this.H        = imageWidthPx;
        this.targetID = targetID;
        // init HuskyLens here…
    }

    /** True if both cameras see the same targetID. */
    public boolean bothSeeTarget() {
        return getLeftID()==targetID && getRightID()==targetID;
    }

    private int getLeftID()  { return targetID; }  // stub
    private int getRightID() { return targetID; }

    /**
     * Compute distance in inches from pixel offsets L,R using stereo formula.
     */
    public double calculateDistance(int L, int R) {
        double rad = Math.PI/180.0;
        double a1 = (L*W/H)+((180-W)/2.0);
        double a2 = (R*W/H)+((180-W)/2.0);
        double denomAng = 180 - (L*W/H + R*W/H) - (180 - W);
        double num = A*Math.sin(a1*rad)*Math.sin(a2*rad);
        double den = Math.sin(denomAng*rad);
        return (den==0) ? -1 : num/den;
    }

    /**
     * Strafing + forward/back to align pixel center & distance.
     * Stubbed to use robot.leftFront, robot.leftBack, etc.
     */
    public void alignToTarget(int pixelTol, double distTol) {
        if (!bothSeeTarget()) return;

        int lc = getLeftCenter(), rc = getRightCenter();
        int avg = (lc+rc)/2;
        int offset = avg - H/2;
        // strafe:
        if (Math.abs(offset)>pixelTol) {
            double p = 0.2 * Math.signum(offset);
            robot.leftFront .setPower(p);
            robot.rightFront.setPower(-p);
            robot.leftBack  .setPower(-p);
            robot.rightBack .setPower(p);
        } else {
            robot.leftFront .setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack  .setPower(0);
            robot.rightBack .setPower(0);
        }

        // distance:
        int L = getLeftOffset(), R = getRightOffset();
        double dist = calculateDistance(L,R);
        if (dist>0 && Math.abs(dist-distTol)>0.5) {
            double p = 0.2 * Math.signum(distDistTol<0 ? -1 : 1);
            robot.leftFront .setPower(p);
            robot.rightFront.setPower(p);
            robot.leftBack  .setPower(p);
            robot.rightBack .setPower(p);
        } else {
            robot.leftFront .setPower(0);
            robot.rightFront.setPower(0);
            robot.leftBack  .setPower(0);
            robot.rightBack .setPower(0);
        }
    }

    // stub helpers:
    private int getLeftCenter()   { return H/2; }
    private int getRightCenter()  { return H/2; }
    private int getLeftOffset()   { return H/4; }
    private int getRightOffset()  { return H/4; }
}
