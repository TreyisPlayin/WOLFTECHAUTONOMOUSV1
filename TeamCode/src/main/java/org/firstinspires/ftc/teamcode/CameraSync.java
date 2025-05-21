package org.firstinspires.ftc.teamcode;

import java.util.Set;

/**
 * CameraSync.java
 *
 * Aligns robot to the same object seen by two HuskyLens cameras,
 * then approaches until within grab distance.
 */
public class CameraSync {
    private final double CENTER_TOL = 20;  // pixels
    private final double GRAB_DIST  = 6.0; // inches
    private final double A          = 7.5; // inches between cams
    private final double FOV        = 60;  // degrees
    private final double IMG_W      = 320; // px

    /** True if both see objectID. */
    public boolean isSameObjectVisible(HuskyLensCamera L, HuskyLensCamera R, int id){
        Set<Integer> l = L.getRecognizedIDs(), r = R.getRecognizedIDs();
        return l.contains(id) && r.contains(id);
    }

    /** Rotate/drive to center and approach object. */
    public void alignToObject(HardwareConfig robot,
                              HuskyLensCamera L, HuskyLensCamera R,
                              int id) {
        while (!robot.leftDrive.isBusy()) {
            if (!isSameObjectVisible(L,R,id)) return;
            int lx=L.getX(id), rx=R.getX(id);
            int center=(lx+rx)/2, off=center-(int)(IMG_W/2);
            double dist=calcDistance(lx,rx);

            if (Math.abs(off)>CENTER_TOL) {
                double p = 0.2*Math.signum(off);
                robot.drive(p, -p);
            } else if (dist>GRAB_DIST) {
                robot.drive(0.2, 0.2);
            } else {
                robot.stopAll();
                break;
            }
        }
    }

    /** Triangulation in degrees as per your formula. */
    public double calcDistance(int LX, int RX) {
        double thetaL = (LX*FOV/IMG_W)+(180-FOV)/2;
        double thetaR = (RX*FOV/IMG_W)+(180-FOV)/2;
        double num = A * Math.sin(Math.toRadians(thetaL)) * Math.sin(Math.toRadians(thetaR));
        double den = Math.sin(Math.toRadians(thetaL+thetaR));
        return num/den;
    }
}
