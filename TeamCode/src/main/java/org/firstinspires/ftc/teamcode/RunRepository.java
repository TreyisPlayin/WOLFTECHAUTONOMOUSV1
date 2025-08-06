package org.firstinspires.ftc.teamcode.ai;

import com.google.gson.Gson;
import com.google.gson.reflect.TypeToken;
import org.firstinspires.ftc.teamcode.Checkpoint;

import java.io.*;
import java.lang.reflect.Type;
import java.util.ArrayList;
import java.util.List;

/**
 * RunRepository saves and loads past autonomous “runs” (presets + performance).
 * Data is serialized as JSON in the robot’s internal storage.
 */
public class RunRepository {
    private static final String FILENAME = "autonomous_runs.json";
    private final Gson gson = new Gson();
    private final File storageDir;

    public RunRepository(File storageDir) {
        this.storageDir = storageDir;
        if (!storageDir.exists()) storageDir.mkdirs();
    }

    /** Data model for a single run record. */
    public static class RunRecord {
        public String presetName;
        public List<Checkpoint> checkpoints;
        public long elapsedTimeMs;
        public double finalX, finalY, finalHeading;
        public double score;
    }

    /**
     * Load all stored runs from disk.
     */
    public List<RunRecord> loadRuns() {
        File file = new File(storageDir, FILENAME);
        if (!file.exists()) return new ArrayList<>();

        try (Reader reader = new FileReader(file)) {
            Type listType = new TypeToken<List<RunRecord>>(){}.getType();
            return gson.fromJson(reader, listType);
        } catch (IOException e) {
            e.printStackTrace();
            return new ArrayList<>();
        }
    }

    /**
     * Save all runs back to disk.
     */
    public void saveRuns(List<RunRecord> runs) {
        File file = new File(storageDir, FILENAME);
        try (Writer writer = new FileWriter(file)) {
            gson.toJson(runs, writer);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Add a new run record and persist.
     */
    public void addRun(RunRecord record) {
        List<RunRecord> runs = loadRuns();
        runs.add(record);
        saveRuns(runs);
    }
}
