package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Small navigation helpers for autonomous.
 * Uses Odometry pose (inches, degrees) to align robot to a field coordinate.
 */
public class NavUtils {

    // === TUNE THESE for your actual far-away scoring goal ===
    // Coordinates are in inches in your 144x144 ZoneMap field frame (0..143).
    public static final double FAR_GOAL_X_IN        = 120.0; // TODO: set real far-goal X
    public static final double FAR_GOAL_Y_IN        = 110.0; // TODO: set real far-goal Y
    public static final double FAR_GOAL_HEADING_DEG =   0.0; // preferred facing when at goal

    /**
     * Rotate in-place until the robot is facing toward (goalX, goalY) within a given tolerance.
     * ONLY rotates; does not drive forward.
     */
    public static void autoAlignToGoal(HardwareConfig robot,
                                       Odometry odo,
                                       double goalX,
                                       double goalY,
                                       double kP,
                                       double headingTolDeg,
                                       long timeoutMs) {
        long start = System.currentTimeMillis();

        while (System.currentTimeMillis() - start < timeoutMs) {
            // Keep odometry fresh
            odo.update();
            Position p = odo.getCurrentPosition();

            // Desired heading (deg) from robot → goal
            double desired = Math.toDegrees(Math.atan2(goalY - p.getY(), goalX - p.getX()));
            double err = angleDiff(desired, p.getHeading());

            if (Math.abs(err) <= headingTolDeg) {
                break;
            }

            double turn = kP * err;
            turn = clamp(turn, -0.4, 0.4); // limit turning power

            double left  =  turn;
            double right = -turn;

            setDrive(robot, left, right);

            try { Thread.sleep(10); } catch (InterruptedException ignored) {}
        }

        stopDrive(robot);
    }

    // === Helpers ===

    /** Angle difference in degrees, wrapped to [-180, 180]. */
    private static double angleDiff(double target, double current) {
        double d = (target - current + 540.0) % 360.0 - 180.0;
        return (d < -180) ? d + 360 : d;
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    private static void setDrive(HardwareConfig robot, double left, double right) {
        robot.leftFront .setPower(left);
        robot.leftBack  .setPower(left);
        robot.rightFront.setPower(right);
        robot.rightBack .setPower(right);
    }

    private static void stopDrive(HardwareConfig robot) {
        setDrive(robot, 0, 0);
    }
}
