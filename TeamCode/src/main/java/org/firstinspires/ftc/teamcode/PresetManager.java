package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/**
 * Build an in-code preset (no UI): zones + ordered checkpoints.
 */
public class PresetManager {
    private final ZoneMap zoneMap = new ZoneMap();
    private final List<Checkpoint> checkpoints = new ArrayList<>();

    /** Mark a rectangular zone on the map. */
    public void addZone(int x0, int y0, int x1, int y1, int zoneType) {
        zoneMap.markZone(x0,y0,x1,y1,zoneType);
    }

    /** Add one checkpoint step. */
    public void addCheckpoint(double x, double y, double heading, String action) {
        checkpoints.add(new Checkpoint(x,y,heading,action));
        zoneMap.markCheckpoint((int)x,(int)y);
    }

    public ZoneMap getZoneMap() { return zoneMap; }
    public List<Checkpoint> getCheckpoints() { return new ArrayList<>(checkpoints); }
}
