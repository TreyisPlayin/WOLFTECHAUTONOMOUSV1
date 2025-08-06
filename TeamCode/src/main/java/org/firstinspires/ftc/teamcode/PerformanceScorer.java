package org.firstinspires.ftc.teamcode.ai;

import org.firstinspires.ftc.teamcode.AutonomousRunner;
import org.firstinspires.ftc.teamcode.Checkpoint;
import org.firstinspires.ftc.teamcode.Position;

import java.util.List;

/**
 * PerformanceScorer evaluates a single autonomous run’s “fitness.”
 * You can customize the scoring formula (time, checkpoints hit, penalties).
 */
public class PerformanceScorer {

    /**
     * Computes a score for a completed run.
     * @param elapsedTimeMs Total time taken (milliseconds)
     * @param finalPosition Robot’s final position
     * @param checkpoints List of checkpoints that were successfully reached
     * @return A higher score is better.
     */
    public static double scoreRun(long elapsedTimeMs,
                                  Position finalPosition,
                                  List<Checkpoint> checkpoints) {
        // Example scoring:
        // - Base score: number of checkpoints reached × 100
        // - Time penalty: subtract elapsedTimeMs / 100
        // - Distance penalty: subtract distance from finalPosition to last checkpoint
        double score = checkpoints.size() * 100.0;
        score -= (elapsedTimeMs / 100.0);

        if (!checkpoints.isEmpty()) {
            Checkpoint last = checkpoints.get(checkpoints.size() - 1);
            double dist = finalPosition.distanceTo(new Position(last.x, last.y, last.heading));
            score -= dist;  // bigger distance from goal lowers score
        }
        return score;
    }
}
