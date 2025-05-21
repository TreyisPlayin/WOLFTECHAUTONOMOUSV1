package org.firstinspires.ftc.teamcode;

import java.util.List;
import org.firstinspires.ftc.robotcore.external.navigation.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;

/**
 * AprilTagTracker.java
 *
 * Uses AprilTag detections to correct odometry drift.
 */
public class AprilTagTracker {
    private AprilTagDetectionPipeline pipeline;
    private VisionPortal portal;
    private Odometry odometry;

    public AprilTagTracker(HardwareConfig robot, Odometry odometry) {
        this.odometry = odometry;
        pipeline = new AprilTagDetectionPipeline();
        portal = VisionPortal.easyCreateWithDefaults(robot.frontSensor.getDeviceInterfaceModule());
        portal.setProcessor(pipeline);
    }

    /** Call each loop to see if a tag is visible and correct pos. */
    public void checkForTags() {
        List<AprilTagDetection> det = pipeline.getDetections();
        if (!det.isEmpty()) {
            AprilTagDetection t = det.get(0);
            odometry.setPosition(new Position((int)t.ftcPose.x, (int)t.ftcPose.y));
        }
    }
}
