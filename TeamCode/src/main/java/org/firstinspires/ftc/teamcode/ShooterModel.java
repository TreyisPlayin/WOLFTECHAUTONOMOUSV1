package org.firstinspires.ftc.teamcode;

/**
 * Shared projectile model used by autonomous and teleop to estimate shooter requirements.
 */
public final class ShooterModel {
    public static final double MAX_SHOOTER_RPM = 5600.0;      // Free speed of the shooter wheel
    public static final double LAUNCH_ANGLE_DEG = 50.0;       // Launch angle of the projectile relative to the floor
    public static final double SHOOTER_WHEEL_RADIUS_IN = 2.0; // Radius of the shooter wheel in inches
    public static final double GRAVITY_IN_PER_S2 = 386.09;    // g in inches/second^2

    private ShooterModel() {
        // Utility class
    }

    public static ShotProfile calculateShot(double distanceIn) {
        double angleRad = Math.toRadians(LAUNCH_ANGLE_DEG);
        double sinTwoTheta = Math.sin(2 * angleRad);

        if (sinTwoTheta <= 1e-6 || SHOOTER_WHEEL_RADIUS_IN <= 0.0 || MAX_SHOOTER_RPM <= 0.0) {
            return new ShotProfile(0.0, 0.0);
        }

        double requiredVelocity = Math.sqrt(Math.max(0.0, distanceIn * GRAVITY_IN_PER_S2 / sinTwoTheta));
        double requiredRpm = (requiredVelocity * 60.0) / (2.0 * Math.PI * SHOOTER_WHEEL_RADIUS_IN);
        double power = clamp(requiredRpm / MAX_SHOOTER_RPM, 0.0, 1.0);

        return new ShotProfile(power, Math.min(requiredRpm, MAX_SHOOTER_RPM));
    }

    public static double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    public static final class ShotProfile {
        public final double power;
        public final double requiredRpm;

        private ShotProfile(double power, double requiredRpm) {
            this.power = power;
            this.requiredRpm = requiredRpm;
        }
    }
}
