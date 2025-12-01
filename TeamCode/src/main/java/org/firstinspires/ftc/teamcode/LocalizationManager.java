package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * LocalizationManager — uses goBILDA pinpoint odometry for continuous pose estimation,
 * plus vision (AprilTag via VisionPortal) as optional correction when a tag is visible.
 */
public class LocalizationManager {
    private final GoBildaPinpointDriver pinpoint;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor tagProcessor;

    // Current pose (mm, radians)
    public double x_mm = 0.0;
    public double y_mm = 0.0;
    public double headingRad = 0.0;

    // internal timer
    private final ElapsedTime timer = new ElapsedTime();

    public LocalizationManager(HardwareMap hardwareMap, GoBildaPinpointDriver pinpoint) {
        this.pinpoint = pinpoint;

        // initialize vision portal + AprilTag
        this.tagProcessor = new AprilTagProcessor.Builder().build();
        this.visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                this.tagProcessor
        );
    }

    /** Must be called once in init (before start) */
    public void start() {
        pinpoint.resetPosAndIMU();
        visionPortal.start();
        timer.reset();
    }

    /** Call this in each loop to update pose estimate */
    public void update() {
        // 1. base pose from pinpoint
        x_mm = pinpoint.getX();
        y_mm = pinpoint.getY();
        headingRad = pinpoint.getHeading(AngleUnit.RADIANS);

        // 2. If vision sees a tag, optionally override / correct pose
        List<AprilTagDetection> det = tagProcessor.getDetections();
        if (det != null && !det.isEmpty()) {
            // choose first detection (or pick the best)
            AprilTagDetection d = det.get(0);
            if (d.ftcPose != null) {
                // get pose of tag relative to camera
                double tagX  = d.ftcPose.x;
                double tagY  = d.ftcPose.y;
                double tagYawRadians = d.ftcPose.yaw;

                // Convert tag-relative pose into field coordinates
                // This requires knowing the physical location of that tag on field,
                // and the transform between camera and robot center. You must set those.
                // For now, as placeholder, just trust the camera-derived pose (approx).
                x_mm = tagX;
                y_mm = tagY;
                headingRad = tagYawRadians;
            }
        }
    }

    /** Get heading in degrees */
    public double getHeadingDeg() {
        return Math.toDegrees(headingRad);
    }

    /** For safety: returns true if vision just saw a tag this cycle */
    public boolean visionUpdateAvailable() {
        List<AprilTagDetection> det = tagProcessor.getDetections();
        return (det != null && !det.isEmpty());
    }
}
