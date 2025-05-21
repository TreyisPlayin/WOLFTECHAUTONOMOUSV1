package org.firstinspires.ftc.teamcode;

/**
 * FieldMapper.java
 *
 * Scales your 24×24 array to a 142×142 grid, preserving proportions.
 * Provides zone lookups and obstacle updates.
 */
public class FieldMapper {
    private static final int FIELD_SIZE = 142;
    private final int rows, cols;
    private final int[][] fieldGrid;

    /**
     * @param smallMap 24×24 zone map from ZoneMap.getXXXZoneMap()
     */
    public FieldMapper(int[][] smallMap) {
        this.rows = smallMap.length;
        this.cols = smallMap[0].length;
        this.fieldGrid = new int[FIELD_SIZE][FIELD_SIZE];
        scaleToFieldSize(smallMap);
    }

    /** Expand each small cell into a proportional block on the 142×142 grid. */
    private void scaleToFieldSize(int[][] smallMap) {
        double scaleX = FIELD_SIZE / (double)rows;
        double scaleY = FIELD_SIZE / (double)cols;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                int zone = smallMap[i][j];
                int x0 = (int)Math.floor(i*scaleX), x1 = (int)Math.ceil((i+1)*scaleX);
                int y0 = (int)Math.floor(j*scaleY), y1 = (int)Math.ceil((j+1)*scaleY);
                for (int x = x0; x < x1; x++) {
                    for (int y = y0; y < y1; y++) {
                        fieldGrid[x][y] = zone;
                    }
                }
            }
        }
    }

    /** Obstacle marking using ultrasonic sensors */
    public void scanWithUltrasonics(HardwareConfig robot, Odometry odometry) {
        Position pos = odometry.getCurrentPosition();
        int cx = pos.x, cy = pos.y;
        // Example for front sensor
        double d = robot.frontSensor.getDistance(com.qualcomm.robotcore.hardware.DistanceUnit.INCH);
        if (d > 0 && d < 30) {
            int ox = Math.min(FIELD_SIZE-1, Math.max(0, cx));
            int oy = Math.min(FIELD_SIZE-1, Math.max(0, cy + (int)Math.round(d)));
            fieldGrid[ox][oy] = -1;
        }
        // Repeat for back/left/right sensors...
    }

    // Zone shortcuts (optional—you can look up by grid value too)
    public Position getStartingZone()     {
        return findZone(ZoneMap.START_ZONE);
    }
    public Position getCollectionZone()   {
        return findZone(ZoneMap.COLLECTION_ZONE);
    }
    public Position getScoringZone()      {
        return findZone(ZoneMap.SCORING_ZONE);
    }
    public Position getHumanPlayerZone()  {
        return findZone(ZoneMap.HUMAN_PLAYER);
    }

    /** Finds the first cell matching a zone type. */
    private Position findZone(int zoneType) {
        for (int x = 0; x < FIELD_SIZE; x++) {
            for (int y = 0; y < FIELD_SIZE; y++) {
                if (fieldGrid[x][y] == zoneType) return new Position(x,y);
            }
        }
        return new Position(0,0); // fallback
    }

    public int[][] getFieldGrid() {
        return fieldGrid;
    }
}
