package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * Simple wrapper to create a VisionPortal with AprilTag processor enabled.
 * Webcam is provided by HardwareConfig.webcam.
 */
public class VisionWebcamPortal {
    public final VisionPortal portal;
    public final AprilTagProcessor april;

    public VisionWebcamPortal(HardwareConfig robot) {
        april = AprilTagProcessor.easyCreateWithDefaults(); // default intrinsics & size (OK for field tags)
        portal = new VisionPortal.Builder()
                .setCamera(robot.webcam)
                .addProcessor(april)
                .setAutoStopLiveView(true)
                .build();

        // Wait until streaming (optional)
        while (portal.getCameraState() != CameraState.STREAMING) {
            try { Thread.sleep(20); } catch (InterruptedException ignored) {}
        }
    }
}
