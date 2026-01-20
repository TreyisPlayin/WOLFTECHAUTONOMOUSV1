package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

public class LimelightTracker {
    private final Limelight3A limelight;
    public static double TARGET_H = 60.0, CAMERA_H = 15.0, MOUNT_A = 25.0; // Inches/Degrees

    public LimelightTracker(Limelight3A limelight) {
        this.limelight = limelight;
        this.limelight.pipelineSwitch(0);
    }

    public boolean hasTarget() {
        LLResult res = limelight.getLatestResult();
        return res!= null && res.isValid();
    }

    public double getDistance() {
        if (!hasTarget()) return 0;
        double ty = limelight.getLatestResult().getTy();
        return (TARGET_H - CAMERA_H) / Math.tan(Math.toRadians(MOUNT_A + ty));
    }

    public double getTX() { return hasTarget()? limelight.getLatestResult().getTx() : 0; }
}