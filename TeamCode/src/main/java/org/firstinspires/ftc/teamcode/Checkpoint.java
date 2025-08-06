package org.firstinspires.ftc.teamcode;

/**
 * A single step in your autonomous sequence.
 * x, y: target coordinates (inches),
 * heading: desired orientation when arriving (degrees),
 * action: label ("start", "pickup", "score", "wait:500", "stop:5", etc.)
 */
public class Checkpoint {
    public final double x, y, heading;
    public final String action;

    public Checkpoint(double x, double y, double heading, String action) {
        this.x       = x;
        this.y       = y;
        this.heading = (heading % 360 + 360) % 360;
        this.action  = action;
    }

    @Override
    public String toString() {
        return String.format("CP[%.1f,%.1f @%.1f° : %s]", x, y, heading, action);
    }
}
