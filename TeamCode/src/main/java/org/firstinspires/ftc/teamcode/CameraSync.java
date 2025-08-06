package org.firstinspires.ftc.teamcode;

/**
 * CameraSync coordinates two front-mounted HuskyLens cameras
 * to confirm they see the same object and to calculate distance.
 * Stubbed here: replace getPixelOffsets() with real SDK calls.
 */
public class CameraSync {
    private final HardwareConfig robot;
    private final double A; // distance between cameras (inches)
    private final double W; // camera FOV (degrees)
    private final int H;    // image width (pixels)
    private final int targetObjectID;

    public CameraSync(HardwareConfig robot,
                      double distanceBetweenCams,
                      double cameraFOV,
                      int imageWidthPixels,
                      int targetObjectID) {
        this.robot = robot;
        this.A = distanceBetweenCams;
        this.W = cameraFOV;
        this.H = imageWidthPixels;
        this.targetObjectID = targetObjectID;

        // Initialize HuskyLens SDK here if needed
    }

    /** Returns true if both cameras see the same target ID. */
    public boolean bothCamerasSeeSame() {
        int leftID  = getLeftCameraID();
        int rightID = getRightCameraID();
        return leftID == targetObjectID && rightID == targetObjectID;
    }

    /** Stub: get tracked ID from left camera */
    private int getLeftCameraID() {
        return targetObjectID; // replace with SDK call
    }

    /** Stub: get tracked ID from right camera */
    private int getRightCameraID() {
        return targetObjectID; // replace with SDK call
    }

    /**
     * Calculate distance to object using pixel offsets L and R
     * and the provided stereo vision formula.
     */
    public double calculateDistance(int L, int R) {
        double rad = Math.PI / 180.0;
        double a1 = (L * W / H) + ((180 - W) / 2);
        double a2 = (R * W / H) + ((180 - W) / 2);
        double denomAng = 180 - (L * W / H + R * W / H) - (180 - W);

        double num = A * Math.sin(a1 * rad) * Math.sin(a2 * rad);
        double den = Math.sin(denomAng * rad);
        if (den == 0) return -1;
        return num / den;
    }

    /**
     * Aligns robot to the target horizontally and in distance.
     * Uses stubbed motor methods: replace robot.drive.* with your drive code.
     */
    public void alignToTarget(int pixelTol, double distTol) {
        if (!bothCamerasSeeSame()) return;

        int leftCenter  = getLeftCenterX();
        int rightCenter = getRightCenterX();
        int avgCenter   = (leftCenter + rightCenter) / 2;
        int offset      = avgCenter - (H / 2);

        // Strafing based on horizontal offset
        if (Math.abs(offset) > pixelTol) {
            if (offset > 0) robot.leftFront.setPower(0.2);
            else           robot.rightFront.setPower(0.2);
        } else {
            robot.leftFront.setPower(0);
            robot.rightFront.setPower(0);
        }

        // Distance control
        int L = getLeftOffsetToEdge();
        int R = getRightOffsetToEdge();
        double dist = calculateDistance(L, R);
        if (dist > 0 && Math.abs(dist - distTol) > 0.5) {
            if (dist > distTol) robot.leftBack.setPower(0.2);
            else                robot.leftBack.setPower(-0.2);
        } else {
            robot.leftBack.setPower(0);
        }
    }

    // Stubbed camera data methods (replace with real HuskyLens SDK):
    private int getLeftCenterX()       { return H/2; }
    private int getRightCenterX()      { return H/2; }
    private int getLeftOffsetToEdge()  { return H/4; }
    private int getRightOffsetToEdge() { return H/4; }
}
