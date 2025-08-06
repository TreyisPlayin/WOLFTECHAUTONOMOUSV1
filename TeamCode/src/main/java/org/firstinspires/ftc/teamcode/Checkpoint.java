package org.firstinspires.ftc.teamcode;

/**
 * A single autonomous step: go to (x,y), face heading°, then perform action.
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
        return String.format("CP[x=%.1f,y=%.1f,hdg=%.1f,act=%s]",
                x, y, heading, action);
    }
}
