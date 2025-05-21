package org.firstinspires.ftc.teamcode;

/**
 * Odometry.java
 *
 * Integrates encoder deltas into X,Y; allows external corrections.
 */
public class Odometry {
    private HardwareConfig robot;
    private Position pos;
    private int lastL=0, lastR=0;

    public Odometry(HardwareConfig robot) {
        this.robot = robot;
        this.pos = new Position(0,0);
        lastL = robot.leftDrive.getCurrentPosition();
        lastR = robot.rightDrive.getCurrentPosition();
    }

    /** Call each loop to update position. */
    public void update() {
        int curL = robot.leftDrive.getCurrentPosition();
        int curR = robot.rightDrive.getCurrentPosition();
        int dl = curL - lastL, dr = curR - lastR;
        lastL = curL; lastR = curR;
        // TODO: convert ticks→inches (with wheel diam & gear ratio)
        double dx = (dl+dr)/2.0; // simplistic
        pos.x += dx; pos.y += 0; // adjust as needed for heading
    }

    public void setPosition(Position corrected) { this.pos = corrected; }
    public Position getCurrentPosition()      { return pos; }
}
