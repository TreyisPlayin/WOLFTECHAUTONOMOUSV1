package org.firstinspires.ftc.teamcode;

import java.util.Arrays;
import java.util.List;

/**
 * Pure Pursuit path follower (mecanum-agnostic power mix).
 * Waypoints are in inches; heading is only used near the goal for final settle.
 *
 * Typical usage:
 *   PurePursuitController pp = new PurePursuitController(robot, odo, 12.0, 0.7, 0.9);
 *   pp.setPath(smoothPath); // List<Position> or Position[]
 *   while (opModeIsActive() && !pp.isFinished()) {
 *       odo.update();
 *       // tagLoc.maybeFuseWithOdometry(odo); // call outside this class
 *       PurePursuitController.Command cmd = pp.update();
 *       pp.applyDrive(cmd);
 *   }
 *   pp.stopDrive();
 */
public class PurePursuitController {

    public static class Command {
        public double vx;     // forward component (robot frame)
        public double vy;     // strafe  component (robot frame)
        public double omega;  // turn component
    }

    private final HardwareConfig robot;
    private final Odometry odo;

    // Tunables
    private double lookaheadIn;       // 8–16" typical
    private double maxTransPower;     // 0..1
    private double maxTurnPower;      // 0..1
    private double kCurv = 0.9;       // path facing feedforward
    private double kHeading = 0.02;   // final heading settle P (deg → power)
    private double finishTolIn = 1.5; // end condition distance

    private Position[] path = new Position[0];
    private int lastClosestIdx = 0;

    public PurePursuitController(HardwareConfig robot, Odometry odo,
                                 double lookaheadIn, double maxTransPower, double maxTurnPower) {
        this.robot = robot;
        this.odo = odo;
        this.lookaheadIn   = lookaheadIn;
        this.maxTransPower = maxTransPower;
        this.maxTurnPower  = maxTurnPower;
    }

    public void setPath(List<Position> pts) {
        this.path = pts.toArray(new Position[0]);
        lastClosestIdx = 0;
    }
    public void setPath(Position[] pts) {
        this.path = Arrays.copyOf(pts, pts.length);
        lastClosestIdx = 0;
    }

    public boolean isFinished() {
        if (path.length == 0) return true;
        Position now = odo.getCurrentPosition();
        Position goal = path[path.length - 1];
        return now.distanceTo(goal) < finishTolIn;
    }

    public Command update() {
        Command out = new Command();
        if (path.length < 2) { stopDrive(); return out; }

        Position rp = odo.getCurrentPosition(); // heading in degrees
        double rx = rp.getX(), ry = rp.getY(), rHdgDeg = rp.getHeading();

        // 1) Closest path index (monotonic search)
        int closest = lastClosestIdx;
        double bestD2 = Double.MAX_VALUE;
        for (int i = lastClosestIdx; i < path.length; i++) {
            double dx = path[i].getX() - rx, dy = path[i].getY() - ry;
            double d2 = dx*dx + dy*dy;
            if (d2 < bestD2) { bestD2 = d2; closest = i; }
        }
        lastClosestIdx = closest;

        // 2) Lookahead point by arc length forward along the path
        Position target = path[Math.min(closest + 1, path.length - 1)];
        double acc = 0;
        for (int i = closest; i < path.length - 1; i++) {
            double seg = path[i].distanceTo(path[i+1]);
            if (acc + seg >= lookaheadIn) {
                double t = (lookaheadIn - acc) / Math.max(seg, 1e-9);
                double lx = lerp(path[i].getX(), path[i+1].getX(), t);
                double ly = lerp(path[i].getY(), path[i+1].getY(), t);
                target = new Position(lx, ly, path[i+1].getHeading());
                break;
            }
            acc += seg;
        }

        // 3) Robot-frame velocity toward lookahead
        double tx = target.getX() - rx, ty = target.getY() - ry;
        double dist = Math.hypot(tx, ty);
        if (dist < 1e-6) { stopDrive(); return out; }

        double h = Math.toRadians(rHdgDeg);
        double c = Math.cos(-h), s = Math.sin(-h);
        double vx = (tx * c - ty * s);
        double vy = (tx * s + ty * c);

        // Normalize and clamp to power
        double norm = Math.max(1.0, dist / lookaheadIn);
        vx = clamp(vx / (lookaheadIn * norm), -maxTransPower, maxTransPower);
        vy = clamp(vy / (lookaheadIn * norm), -maxTransPower, maxTransPower);

        // 4) Curvature feedforward (face next segment)
        double omegaFF = 0.0;
        if (closest < path.length - 1) {
            double dx = path[closest+1].getX() - path[closest].getX();
            double dy = path[closest+1].getY() - path[closest].getY();
            double segHdgDeg = Math.toDegrees(Math.atan2(dy, dx));
            double errDeg = angleWrapDeg(segHdgDeg - rHdgDeg);
            omegaFF = kCurv * (errDeg / 90.0); // ±90° → ±kCurv
        }

        // 5) Final heading settle near the goal
        double omegaP = 0.0;
        Position goal = path[path.length - 1];
        if (dist < 2 * lookaheadIn) {
            double errDeg = angleWrapDeg(goal.getHeading() - rHdgDeg);
            omegaP = kHeading * errDeg;
        }

        out.vx = vx;
        out.vy = vy;
        out.omega = clamp(omegaFF + omegaP, -maxTurnPower, maxTurnPower);
        return out;
    }

    public void applyDrive(Command cmd) {
        // Mecanum mix (VX forward, VY left, OMEGA CCW positive)
        double lf = cmd.vx + cmd.vy + cmd.omega;
        double rf = cmd.vx - cmd.vy - cmd.omega;
        double lb = cmd.vx - cmd.vy + cmd.omega;
        double rb = cmd.vx + cmd.vy - cmd.omega;
        double max = Math.max(1.0, Math.max(Math.abs(lf),
                Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb)))));
        robot.leftFront .setPower(lf / max);
        robot.rightFront.setPower(rf / max);
        robot.leftBack  .setPower(lb / max);
        robot.rightBack .setPower(rb / max);
    }

    public void stopDrive() {
        robot.leftFront .setPower(0);
        robot.rightFront.setPower(0);
        robot.leftBack  .setPower(0);
        robot.rightBack .setPower(0);
    }

    // Tuners
    public void setLookahead(double in)         { this.lookaheadIn = in; }
    public void setFinishTolerance(double in)   { this.finishTolIn = in; }
    public void setCurvatureFF(double k)        { this.kCurv = k; }
    public void setHeadingP(double k)           { this.kHeading = k; }

    // Helpers
    private static double lerp(double a, double b, double t) { return a + (b - a) * t; }
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double angleWrapDeg(double a){
        a = (a % 360 + 360) % 360; if (a > 180) a -= 180*2;
        return a;
    }
}
