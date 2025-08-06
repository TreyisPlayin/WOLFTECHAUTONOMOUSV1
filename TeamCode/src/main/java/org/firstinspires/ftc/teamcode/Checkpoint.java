package org.firstinspires.ftc.teamcode;

/**
 * Checkpoint defines a single step in your autonomous sequence.
 * - x, y: target field coordinates (inches)
 * - heading: robot orientation when arriving (degrees)
 * - action: label for what to do at this point (e.g., "pickup", "score", "wait:500")
 */
public class Checkpoint {
    public final double x;
    public final double y;
    public final double heading;
    public final String action;

    /**
     * @param x        X-coordinate in inches   (0–144)
     * @param y        Y-coordinate in inches   (0–144)
     * @param heading  Desired robot heading in degrees (0–360)
     * @param action   Action label or parameters
     */
    public Checkpoint(double x, double y, double heading, String action) {
        this.x       = x;
        this.y       = y;
        this.heading = (heading % 360 + 360) % 360;
        this.action  = action;
    }

    @Override
    public String toString() {
        return String.format("Checkpoint[x=%.1f,y=%.1f,h=%.1f° action=%s]",
                x, y, heading, action);
    }
}
