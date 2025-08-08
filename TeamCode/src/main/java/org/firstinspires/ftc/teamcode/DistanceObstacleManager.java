package org.firstinspires.ftc.teamcode;

/**
 * Reads 4 DistanceSensors and projects obstacles into the ZoneMap around the robot.
 * Inflates obstacles by half the robot width so A* treats them as no-go.
 */
public class DistanceObstacleManager {
    private final HardwareConfig robot;
    private final ZoneMap map;
    private final double robotHalf = 9.0; // 18x18 robot → inflate by ~9"

    public DistanceObstacleManager(HardwareConfig hw, ZoneMap map) {
        this.robot = hw;
        this.map   = map;
    }

    /** Call periodically during auto to mark obstacles as FORBIDDEN. */
    public void updateObstacles(Position pose) {
        // Simple projection: convert each sensor distance to a (x,y) point in field coords.
        markIfValid(projectPoint(pose,  0, robot.distFront.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
        markIfValid(projectPoint(pose,180, robot.distBack .getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
        markIfValid(projectPoint(pose, 90, robot.distLeft .getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
        markIfValid(projectPoint(pose,270, robot.distRight.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
    }

    private double[] projectPoint(Position pose, double sensorBearingDeg, double distIn) {
        if (distIn <= 0 || distIn > 48) return null; // ignore invalid/far
        double globalBearing = Math.toRadians(pose.getHeading() + sensorBearingDeg);
        double x = pose.getX() + (distIn + robotHalf) * Math.cos(globalBearing);
        double y = pose.getY() + (distIn + robotHalf) * Math.sin(globalBearing);
        return new double[]{x,y};
    }

    private void markIfValid(double[] xy) {
        if (xy == null) return;
        int xi = (int)Math.round(xy[0]);
        int yi = (int)Math.round(xy[1]);
        if (xi>=0 && yi>=0 && xi<ZoneMap.SIZE && yi<ZoneMap.SIZE) {
            map.markZone(Math.max(0,xi-1), Math.max(0,yi-1), Math.min(ZoneMap.SIZE-1,xi+1), Math.min(ZoneMap.SIZE-1,yi+1), ZoneMap.FORBIDDEN);
        }
    }
}
