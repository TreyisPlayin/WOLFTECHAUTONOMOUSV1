package org.firstinspires.ftc.teamcode.ai;

import org.firstinspires.ftc.teamcode.Checkpoint;
import org.firstinspires.ftc.teamcode.Position;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * GenerationTrainer manages evolutionary training of presets:
 *  - Runs multiple “variants” per generation
 *  - Scores each
 *  - Selects the top N to seed the next generation via mutation
 */
public class GenerationTrainer {

    private final org.firstinspires.ftc.teamcode.ai.RunRepository repo;
    private final int populationSize;
    private final int eliteCount;
    private final double mutationRate;

    public GenerationTrainer(File storageDir,
                             int populationSize,
                             int eliteCount,
                             double mutationRate) {
        this.repo          = new org.firstinspires.ftc.teamcode.ai.RunRepository(storageDir);
        this.populationSize= populationSize;
        this.eliteCount   = eliteCount;
        this.mutationRate = mutationRate;
    }

    /**
     * Perform one generation:
     *  - Load past runs
     *  - Keep top elites
     *  - Mutate elites to refill population
     */
    public List<org.firstinspires.ftc.teamcode.ai.RunRepository.RunRecord> evolve() {
        // Load all past run records
        List<org.firstinspires.ftc.teamcode.ai.RunRepository.RunRecord> runs = repo.loadRuns();
        // Sort descending by score
        runs.sort(Comparator.comparingDouble(r -> -r.score));

        // Keep elites
        List<org.firstinspires.ftc.teamcode.ai.RunRepository.RunRecord> nextGen = new ArrayList<>();
        for (int i = 0; i < Math.min(eliteCount, runs.size()); i++) {
            nextGen.add(runs.get(i));
        }

        // Mutate elites to create new variants
        while (nextGen.size() < populationSize) {
            org.firstinspires.ftc.teamcode.ai.RunRepository.RunRecord parent = nextGen.get(nextGen.size() % eliteCount);
            org.firstinspires.ftc.teamcode.ai.RunRepository.RunRecord child = mutate(parent);
            nextGen.add(child);
        }

        // Persist next generation as new runs
        repo.saveRuns(nextGen);
        return nextGen;
    }

    /**
     * Applies mutation to a run’s checkpoints:
     * small random shifts in positions or random action tweaks.
     */
    private org.firstinspires.ftc.teamcode.ai.RunRepository.RunRecord mutate(org.firstinspires.ftc.teamcode.ai.RunRepository.RunRecord orig) {
        org.firstinspires.ftc.teamcode.ai.RunRepository.RunRecord copy = new org.firstinspires.ftc.teamcode.ai.RunRepository.RunRecord();
        copy.presetName   = orig.presetName + "_mut";
        copy.checkpoints  = new ArrayList<>();
        // Mutate each checkpoint
        for (Checkpoint cp : orig.checkpoints) {
            double nx = cp.x + (Math.random()*2-1)*mutationRate;   // ±mutationRate inches
            double ny = cp.y + (Math.random()*2-1)*mutationRate;
            double nh = cp.heading + (Math.random()*2-1)*mutationRate*10; // ±10×rate degrees
            String na = cp.action; // you could randomly swap actions too
            copy.checkpoints.add(new Checkpoint(nx, ny, nh, na));
        }
        // Initialize timing and score to zero; will be set when run later
        copy.elapsedTimeMs = 0;
        copy.finalX = copy.finalY = copy.finalHeading = 0;
        copy.score = 0;
        return copy;
    }
}
