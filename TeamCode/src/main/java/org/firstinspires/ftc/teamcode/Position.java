package org.firstinspires.ftc.teamcode;

/**
 * Holds (x, y) in inches and heading in degrees.
 */
public class Position {
    private double x, y, heading;

    public Position(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        setHeading(heading);
    }

    public double getX()       { return x; }
    public double getY()       { return y; }
    public double getHeading() { return heading; }

    public void setHeading(double hdg) {
        this.heading = (hdg % 360 + 360) % 360;
    }

    /** Euclidean distance to another pos. */
    public double distanceTo(Position o) {
        double dx = o.x - x, dy = o.y - y;
        return Math.hypot(dx, dy);
    }

    @Override public String toString() {
        return String.format("Pos[%.1f,%.1f @%.1f°]", x, y, heading);
    }
}
