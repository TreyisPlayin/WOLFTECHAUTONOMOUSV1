package org.firstinspires.ftc.teamcode;

import java.util.Arrays;

/**
 * ZoneMap represents the FTC field as a 144×144 grid (1" per cell).
 * You can mark rectangular zones (start, scoring, forbidden, etc.)
 * or individual checkpoints directly on this grid.
 */
public class ZoneMap {
    public static final int SIZE = 144;

    // Cell type constants
    public static final int EMPTY         = 0;
    public static final int START_ZONE    = 1;
    public static final int SCORING_ZONE  = 2;
    public static final int CHECKPOINT    = 3;
    public static final int FORBIDDEN     = 4;
    // Add more types as needed

    private final int[][] grid = new int[SIZE][SIZE];

    /** Initialize all cells to EMPTY. */
    public ZoneMap() {
        for (int y = 0; y < SIZE; y++) {
            Arrays.fill(grid[y], EMPTY);
        }
    }

    /**
     * Mark a rectangular region on the field.
     * @param x0      left X in inches (0–143)
     * @param y0      bottom Y in inches (0–143)
     * @param x1      right X in inches
     * @param y1      top Y in inches
     * @param zoneType one of the constants above
     */
    public void markZone(int x0, int y0, int x1, int y1, int zoneType) {
        for (int y = y0; y <= y1; y++) {
            for (int x = x0; x <= x1; x++) {
                if (inBounds(x,y)) {
                    grid[y][x] = zoneType;
                }
            }
        }
    }

    /**
     * Mark a single cell as a checkpoint.
     * This cell will also be stored in PresetManager as a step.
     */
    public void markCheckpoint(int x, int y) {
        if (inBounds(x,y)) {
            grid[y][x] = CHECKPOINT;
        }
    }

    /** Retrieve the zone type at a given cell. */
    public int getCell(int x, int y) {
        return inBounds(x,y) ? grid[y][x] : FORBIDDEN;
    }

    /** Returns true if the coordinates are within [0, SIZE). */
    private boolean inBounds(int x, int y) {
        return x >= 0 && y >= 0 && x < SIZE && y < SIZE;
    }

    /** Expose the raw grid for pathfinding or UI. */
    public int[][] getGrid() {
        return grid;
    }
}
