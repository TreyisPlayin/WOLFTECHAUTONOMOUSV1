package org.firstinspires.ftc.teamcode;

/**
 * Syncs two front HuskyLens cameras to align/triangulate targets.
 * Replace stub methods with your HuskyLens SDK calls.
 */
public class CameraSync {
    private final HardwareConfig robot;
    private final double A, W;
    private final int H;
    private final int targetID;

    public CameraSync(HardwareConfig robot,
                      double distBetweenCams,
                      double camFOV,
                      int imageWidth,
                      int targetID) {
        this.robot    = robot;
        this.A        = distBetweenCams;
        this.W        = camFOV;
        this.H        = imageWidth;
        this.targetID = targetID;
    }

    public boolean bothSeeTarget() {
        return getLeftID()==targetID && getRightID()==targetID;
    }

    private int getLeftID()  { return targetID; }  // stub
    private int getRightID() { return targetID; }

    public double calculateDistance(int L, int R) {
        double rad = Math.PI/180.0;
        double a1 = (L*W/H)+((180-W)/2), a2 = (R*W/H)+((180-W)/2);
        double denom = 180 - (L*W/H + R*W/H) - (180 - W);
        double num = A*Math.sin(a1*rad)*Math.sin(a2*rad);
        double d = Math.sin(denom*rad);
        return d==0 ? -1 : num/d;
    }

    public void alignToTarget(int pixelTol, double distTol) {
        if (!bothSeeTarget()) return;
        int lc = getLeftCenter(), rc = getRightCenter();
        int avg = (lc+rc)/2;
        int offset = avg - H/2;
        if (Math.abs(offset)>pixelTol) {
            if (offset>0) robot.leftFront.setPower(0.2);
            else          robot.rightFront.setPower(0.2);
        } else {
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
        }
        int L=getLeftEdgeOffset(), R=getRightEdgeOffset();
        double dist = calculateDistance(L,R);
        if (dist>0 && Math.abs(dist-distTol)>0.5) {
            if (dist>distTol) robot.leftBack.setPower(0.2);
            else              robot.leftBack.setPower(-0.2);
        } else robot.leftBack.setPower(0);
    }

    private int getLeftCenter()       { return H/2; }
    private int getRightCenter()      { return H/2; }
    private int getLeftEdgeOffset()   { return H/4; }
    private int getRightEdgeOffset()  { return H/4; }
}
