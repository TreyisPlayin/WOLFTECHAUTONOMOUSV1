package org.firstinspires.ftc.teamcode;

import java.util.Arrays;

/**
 * Represents the 144×144 FTC field as a 1"-per-cell grid.
 * You can mark rectangular zones or individual checkpoint cells.
 */
public class ZoneMap {
    public static final int SIZE = 144;

    public static final int EMPTY        = 0;
    public static final int START_ZONE   = 1;
    public static final int SCORING_ZONE = 2;
    public static final int CHECKPOINT   = 3;
    public static final int FORBIDDEN    = 4;
    // extend with more types if needed

    private final int[][] grid = new int[SIZE][SIZE];

    public ZoneMap() {
        for (int y = 0; y < SIZE; y++) {
            Arrays.fill(grid[y], EMPTY);
        }
    }

    /** Mark a rectangular zone from (x0,y0) to (x1,y1) as zoneType. */
    public void markZone(int x0, int y0, int x1, int y1, int zoneType) {
        for (int y = Math.max(0,y0); y <= Math.min(y1,SIZE-1); y++) {
            for (int x = Math.max(0,x0); x <= Math.min(x1,SIZE-1); x++) {
                grid[y][x] = zoneType;
            }
        }
    }

    /** Mark a single cell as a checkpoint. */
    public void markCheckpoint(int x, int y) {
        if (x >= 0 && x < SIZE && y >= 0 && y < SIZE) {
            grid[y][x] = CHECKPOINT;
        }
    }

    /** Get the zone type at (x,y). */
    public int getCell(int x, int y) {
        if (x < 0 || y < 0 || x >= SIZE || y >= SIZE) return FORBIDDEN;
        return grid[y][x];
    }

    /** Expose the raw grid. */
    public int[][] getGrid() {
        return grid;
    }
}
