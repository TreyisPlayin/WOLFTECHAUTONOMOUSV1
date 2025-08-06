package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/**
 * Manages checkpoints and zone-actions for one preset.
 * Now supports editing existing checkpoints by grid cell.
 */
public class PresetManager {
    private final List<Checkpoint> checkpoints = new ArrayList<>();
    private final List<ZoneAction> zoneActions = new ArrayList<>();

    /** Add a new checkpoint */
    public void addCheckpoint(double x, double y, double heading, String action) {
        checkpoints.add(new Checkpoint(x, y, heading, action));
    }

    /** Update an existing checkpoint at index */
    public void updateCheckpoint(int index, double x, double y, double heading, String action) {
        if (index >= 0 && index < checkpoints.size()) {
            checkpoints.set(index, new Checkpoint(x, y, heading, action));
        }
    }

    /** Find the index of a checkpoint whose (int)x,int)y matches the given cell, or -1 */
    public int findCheckpointIndexAt(int cellX, int cellY) {
        for (int i = 0; i < checkpoints.size(); i++) {
            Checkpoint cp = checkpoints.get(i);
            if ((int)cp.x == cellX && (int)cp.y == cellY) {
                return i;
            }
        }
        return -1;
    }

    /** Remove all checkpoints */
    public void clearCheckpoints() {
        checkpoints.clear();
    }

    /** Get a copy of the checkpoints */
    public List<Checkpoint> getCheckpoints() {
        return new ArrayList<>(checkpoints);
    }

    /** Zone-action API unchanged */
    public void addZoneAction(int x0,int y0,int x1,int y1,String action) {
        zoneActions.add(new ZoneAction(x0,y0,x1,y1,action));
    }
    public List<ZoneAction> getZoneActions() {
        return new ArrayList<>(zoneActions);
    }
    public static class ZoneAction { /* unchanged */ }
}
