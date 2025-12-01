package org.firstinspires.ftc.teamcode;

import java.util.Arrays;

/**
 * BallInventory — tracks artifacts inside the robot.
 *
 * channels[0] = LEFT channel
 * channels[1] = RIGHT channel
 *
 * slots ordered front (0) → back (2)
 */
public class BallInventory {

    public static final int NUM_CHANNELS = 2;
    public static final int SLOTS_PER_CHANNEL = 2;

    private final ArtifactColor[][] slots =
            new ArtifactColor[NUM_CHANNELS][SLOTS_PER_CHANNEL];

    private int activeChannel = 0;

    public BallInventory() {
        clearAll();
    }

    public void clearAll() {
        for (int c = 0; c < NUM_CHANNELS; c++) {
            for (int s = 0; s < SLOTS_PER_CHANNEL; s++) {
                slots[c][s] = ArtifactColor.UNKNOWN;
            }
        }
    }

    public void setActiveChannel(int channel) {
        if (channel < 0 || channel >= NUM_CHANNELS) return;
        activeChannel = channel;
    }

    public int getActiveChannel() {
        return activeChannel;
    }

    public void seedPreset(ArtifactColor[] preset) {
        for (int i = 0; i < SLOTS_PER_CHANNEL; i++) {
            if (i < preset.length) slots[activeChannel][i] = preset[i];
            else slots[activeChannel][i] = ArtifactColor.UNKNOWN;
        }
    }

    public void insertAtBack(int channel, ArtifactColor color) {
        if (channel < 0 || channel >= NUM_CHANNELS) return;
        for (int i = 0; i < SLOTS_PER_CHANNEL - 1; i++) {
            slots[channel][i] = slots[channel][i + 1];
        }
        slots[channel][SLOTS_PER_CHANNEL - 1] = color;
    }

    public void popFront(int channel) {
        if (channel < 0 || channel >= NUM_CHANNELS) return;
        for (int i = 0; i < SLOTS_PER_CHANNEL - 1; i++) {
            slots[channel][i] = slots[channel][i + 1];
        }
        slots[channel][SLOTS_PER_CHANNEL - 1] = ArtifactColor.UNKNOWN;
    }

    public ArtifactColor peekFront(int channel) {
        if (channel < 0 || channel >= NUM_CHANNELS) return ArtifactColor.UNKNOWN;
        return slots[channel][0];
    }

    public ArtifactColor[][] getSnapshot() {
        ArtifactColor[][] copy = new ArtifactColor[NUM_CHANNELS][SLOTS_PER_CHANNEL];
        for (int c = 0; c < NUM_CHANNELS; c++) {
            copy[c] = Arrays.copyOf(slots[c], SLOTS_PER_CHANNEL);
        }
        return copy;
    }
}
