package org.firstinspires.ftc.teamcode.ai;

import org.firstinspires.ftc.teamcode.Checkpoint;
import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Random;

/**
 * GenerationTrainer manages an evolutionary loop over past autonomous runs:
 *  - Keeps the top N “elite” records by score
 *  - Mutates them to refill the population
 *  - Persists the next generation back to disk
 */
public class GenerationTrainer {
    private final org.firstinspires.ftc.teamcode.ai.RunRepository repo;
    private final int populationSize;
    private final int eliteCount;
    private final double mutationRate;
    private final Random rng = new Random();

    /**
     * @param storageDir     directory under which runs.json is stored
     * @param populationSize total runs to keep each generation
     * @param eliteCount     how many top performers to carry over unchanged
     * @param mutationRate   maximum ±inches perturbation for each checkpoint
     */
    public GenerationTrainer(File storageDir,
                             int populationSize,
                             int eliteCount,
                             double mutationRate) {
        this.repo           = new org.firstinspires.ftc.teamcode.ai.RunRepository(storageDir);
        this.populationSize = populationSize;
        this.eliteCount     = eliteCount;
        this.mutationRate   = mutationRate;
    }

    /**
     * Perform one evolutionary step:
     * 1) load all past RunRecords
     * 2) sort by descending score
     * 3) keep the top eliteCount as-is
     * 4) mutate elites to fill up to populationSize
     * 5) save the next generation over the old file
     *
     * @return the list of RunRecords in the new generation
     */
    public List<RunRepository.RunRecord> evolve() {
        // 1. load and sort
        List<RunRepository.RunRecord> runs = repo.loadRuns();
        runs.sort(Comparator.comparingDouble(r -> -r.score));

        // 2. keep elites
        List<RunRepository.RunRecord> nextGen = new ArrayList<>();
        for (int i = 0; i < Math.min(eliteCount, runs.size()); i++) {
            nextGen.add(runs.get(i));
        }

        // 3. mutate to fill population
        while (nextGen.size() < populationSize) {
            // pick a random elite to mutate
            RunRepository.RunRecord parent = nextGen.get(rng.nextInt(nextGen.size()));
            nextGen.add(mutate(parent));
        }

        // 4. persist and return
        repo.saveRuns(nextGen);
        return nextGen;
    }

    /**
     * Create a mutated copy of a RunRecord:
     * - Perturb each checkpoint’s x/y by ±mutationRate inches
     * - Perturb heading by ±(mutationRate*10) degrees
     * - Keep action labels the same
     * - Reset timing & score to 0 to be re-evaluated on next run
     */
    private RunRepository.RunRecord mutate(RunRepository.RunRecord orig) {
        RunRepository.RunRecord copy = new RunRepository.RunRecord();
        copy.presetName   = orig.presetName + "_m";
        copy.checkpoints  = new ArrayList<>();

        for (Checkpoint cp : orig.checkpoints) {
            double nx = cp.x + (rng.nextDouble() * 2 - 1) * mutationRate;
            double ny = cp.y + (rng.nextDouble() * 2 - 1) * mutationRate;
            double nh = cp.heading + (rng.nextDouble() * 2 - 1) * mutationRate * 10;
            copy.checkpoints.add(new Checkpoint(nx, ny, nh, cp.action));
        }

        // Reset performance metrics
        copy.elapsedMs    = 0L;
        copy.finalX       = 0.0;
        copy.finalY       = 0.0;
        copy.finalHeading = 0.0;
        copy.score        = 0.0;

        return copy;
    }
}
