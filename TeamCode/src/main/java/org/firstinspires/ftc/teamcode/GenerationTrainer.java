package org.firstinspires.ftc.teamcode.ai;

import org.firstinspires.ftc.teamcode.Checkpoint;
import java.io.File;
import java.util.*;

/**
 * Evolves new generations of presets via mutation/crossover.
 */
public class GenerationTrainer {
    private final RunRepository repo;
    private final int popSize, eliteCount;
    private final double mutationRate;

    public GenerationTrainer(File storageDir,
                             int popSize,
                             int eliteCount,
                             double mutationRate) {
        this.repo = new RunRepository(storageDir);
        this.popSize = popSize;
        this.eliteCount = eliteCount;
        this.mutationRate = mutationRate;
    }

    public List<RunRepository.RunRecord> evolve() {
        List<RunRepository.RunRecord> runs = repo.loadRuns();
        runs.sort(Comparator.comparingDouble(r->-r.score));
        List<RunRepository.RunRecord> next = new ArrayList<>();

        // keep elites
        for(int i=0;i<Math.min(eliteCount,runs.size());i++){
            next.add(runs.get(i));
        }
        // mutate to fill pop
        Random rng=new Random();
        while(next.size()<popSize){
            RunRepository.RunRecord p = next.get(rng.nextInt(eliteCount));
            next.add(mutate(p));
        }
        repo.saveRuns(next);
        return next;
    }

    private RunRepository.RunRecord mutate(RunRepository.RunRecord p) {
        RunRepository.RunRecord c=new RunRepository.RunRecord();
        c.presetName = p.presetName + "_m";
        c.checkpoints = new ArrayList<>();
        for(Checkpoint cp: p.checkpoints){
            double nx = cp.x + (rng()-0.5)*2*mutationRate;
            double ny = cp.y + (rng()-0.5)*2*mutationRate;
            double nh = cp.heading + (rng()-0.5)*2*mutationRate*10;
            c.checkpoints.add(new Checkpoint(nx,ny,nh,cp.action));
        }
        c.elapsedMs = c.finalX = c.finalY = c.finalHeading = c.score = 0;
        return c;
    }
    private double rng(){ return Math.random(); }
}
