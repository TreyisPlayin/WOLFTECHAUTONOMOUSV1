package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.List;

/**
 * PresetManager holds an ordered list of checkpoints (a single “preset”).
 * You can add, clear or retrieve that list for your autonomous runner.
 */
public class PresetManager {
    private final List<Checkpoint> checkpoints = new ArrayList<>();

    /** Add a new checkpoint to the end of the preset. */
    public void addCheckpoint(double x, double y, double heading, String action) {
        checkpoints.add(new Checkpoint(x, y, heading, action));
    }

    /** Remove all checkpoints (start over). */
    public void clearCheckpoints() {
        checkpoints.clear();
    }

    /** Get the immutable list of defined checkpoints. */
    public List<Checkpoint> getCheckpoints() {
        return new ArrayList<>(checkpoints);
    }

    /** How many steps have been defined. */
    public int size() {
        return checkpoints.size();
    }
}
