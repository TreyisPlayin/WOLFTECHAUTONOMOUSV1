package org.firstinspires.ftc.teamcode.ai;

import com.google.gson.*;
import com.google.gson.reflect.TypeToken;
import org.firstinspires.ftc.teamcode.*;
import java.io.*;
import java.lang.reflect.Type;
import java.util.*;

/**
 * Saves/loads autonomous run records as JSON.
 */
public class RunRepository {
    private static final String FNAME="autonomous_runs.json";
    private final Gson gson=new Gson();
    private final File dir;

    public RunRepository(File storageDir) {
        this.dir=storageDir;
        if (!dir.exists()) dir.mkdirs();
    }

    public static class RunRecord {
        public String presetName;
        public List<Checkpoint> checkpoints;
        public long elapsedMs;
        public double finalX, finalY, finalHeading, score;
    }

    public List<RunRecord> loadRuns() {
        File f=new File(dir,FNAME);
        if (!f.exists()) return new ArrayList<>();
        try (Reader r=new FileReader(f)) {
            Type t=new TypeToken<List<RunRecord>>(){}.getType();
            return gson.fromJson(r,t);
        } catch(Exception e){ e.printStackTrace(); return new ArrayList<>(); }
    }

    public void saveRuns(List<RunRecord> runs) {
        try (Writer w=new FileWriter(new File(dir,FNAME))) {
            gson.toJson(runs,w);
        } catch(Exception e) { e.printStackTrace(); }
    }

    public void addRun(RunRecord rec) {
        List<RunRecord> all=loadRuns();
        all.add(rec);
        saveRuns(all);
    }
}
