package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Uses AprilTag detections to compute the robot's global field pose and
 * gently blend into Odometry to correct drift.
 *
 * Math (2D SE(2)):
 *   camInTag     = inverse(tagInCam from detection.ftcPose)
 *   camInField   = tagInField (known) ∘ camInTag
 *   robotInField = camInField ∘ camToRobot (fixed camera offset on robot)
 */
public class AprilTagLocalizer {

    /** Known AprilTag field poses (center of tag) in inches & degrees. */
    public static class FieldTag {
        public final int id;
        public final double xIn, yIn, headingDeg;
        public FieldTag(int id, double xIn, double yIn, double headingDeg) {
            this.id = id; this.xIn = xIn; this.yIn = yIn; this.headingDeg = headingDeg;
        }
    }

    /** Fixed camera-to-robot transform (2D) in inches & degrees. */
    public static class CamToRobot {
        public final double xIn, yIn, headingDeg;
        public CamToRobot(double xIn, double yIn, double headingDeg) {
            this.xIn = xIn; this.yIn = yIn; this.headingDeg = headingDeg;
        }
    }

    private final AprilTagProcessor april;
    private final Map<Integer, FieldTag> fieldTags = new HashMap<>();
    private final CamToRobot camToRobot;

    // Visibility/quality filters
    private double maxTagDistanceIn = 120.0; // ignore farther than 10 ft
    private double maxYawErrorDeg    = 30.0; // ignore steep yaw

    public AprilTagLocalizer(AprilTagProcessor april, CamToRobot camToRobot) {
        this.april = april;
        this.camToRobot = camToRobot;
    }

    public void addFieldTag(int id, double xIn, double yIn, double headingDeg) {
        fieldTags.put(id, new FieldTag(id, xIn, yIn, headingDeg));
    }

    public void setFilters(double maxDistanceIn, double maxYawDeg) {
        this.maxTagDistanceIn = maxDistanceIn;
        this.maxYawErrorDeg   = maxYawDeg;
    }

    /**
     * If a good tag is visible, compute robotInField and blend into odometry.
     * Returns the pose applied to odometry (null if no usable detection).
     */
    public Position maybeFuseWithOdometry(Odometry odo) {
        List<AprilTagDetection> dets = april.getDetections();
        if (dets == null || dets.isEmpty()) return null;

        // Pick best detection (closest)
        AprilTagDetection best = null;
        double bestDist = Double.MAX_VALUE;
        for (AprilTagDetection d : dets) {
            if (!fieldTags.containsKey(d.id)) continue;
            double dist = Math.hypot(d.ftcPose.x, d.ftcPose.y);
            if (dist < bestDist) { bestDist = dist; best = d; }
        }
        if (best == null || bestDist > maxTagDistanceIn) return null;
        if (Math.abs(best.ftcPose.yaw) > maxYawErrorDeg) return null;

        FieldTag tagF = fieldTags.get(best.id);

        // tagInCam (from detection)
        Pose2D tagInCam = new Pose2D(best.ftcPose.x, best.ftcPose.y, best.ftcPose.yaw);
        // camInTag = inverse(tagInCam)
        Pose2D camInTag = tagInCam.inverse();
        // tagInField is known
        Pose2D tagInField = new Pose2D(tagF.xIn, tagF.yIn, tagF.headingDeg);
        // camInField = tagInField ∘ camInTag
        Pose2D camInField = tagInField.compose(camInTag);
        // robotInField = camInField ∘ camToRobot
        Pose2D robotInField = camInField.compose(new Pose2D(camToRobot.xIn, camToRobot.yIn, camToRobot.headingDeg));

        // Convert to Position (deg heading)
        Position tagPose = new Position(robotInField.x, robotInField.y, robotInField.headingDeg);

        // --- Blending weight from distance & yaw quality (0..1) ---
        double distW = Math.max(0, 1.0 - (bestDist / maxTagDistanceIn));  // nearer => stronger
        double yawW  = Math.max(0, 1.0 - (Math.abs(best.ftcPose.yaw) / maxYawErrorDeg));
        double w     = Math.min(1.0, 0.5*distW + 0.5*yawW);

        // Current odometry pose
        Position odomPose = odo.getCurrentPosition(); // deg
        double bx = odomPose.getX(), by = odomPose.getY(), bh = odomPose.getHeading();

        // Blend x,y linearly; heading via shortest-arc delta
        double nx = bx*(1.0 - w) + tagPose.getX()*w;
        double ny = by*(1.0 - w) + tagPose.getY()*w;
        double dh = ((tagPose.getHeading() - bh + 540) % 360) - 180;
        double nh = bh + w * dh;
        Position blended = new Position(nx, ny, nh);

        if (w > 0.85) {
            // Very confident → allow crisp correction
            odo.setPose(tagPose);   // external correction hook in your Odometry
            return tagPose;
        } else {
            odo.setPose(blended);
            return blended;
        }
    }

    // ---- simple 2D rigid transform (inches & degrees) ----
    private static class Pose2D {
        final double x, y, headingDeg;
        Pose2D(double x, double y, double headingDeg) {
            this.x = x; this.y = y; this.headingDeg = normDeg(headingDeg);
        }
        Pose2D compose(Pose2D b) {
            double th = Math.toRadians(this.headingDeg);
            double cx = this.x +  Math.cos(th)*b.x - Math.sin(th)*b.y;
            double cy = this.y +  Math.sin(th)*b.x + Math.cos(th)*b.y;
            double ch = normDeg(this.headingDeg + b.headingDeg);
            return new Pose2D(cx, cy, ch);
        }
        Pose2D inverse() {
            double th = Math.toRadians(this.headingDeg);
            double ix = -( Math.cos(th)*this.x + Math.sin(th)*this.y);
            double iy = -(-Math.sin(th)*this.x + Math.cos(th)*this.y);
            double ih = normDeg(-this.headingDeg);
            return new Pose2D(ix, iy, ih);
        }
        static double normDeg(double a) {
            a = (a % 360 + 360) % 360;
            return a;
        }
    }
}
