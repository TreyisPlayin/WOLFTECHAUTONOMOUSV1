package org.firstinspires.ftc.teamcode.ai;

import org.firstinspires.ftc.teamcode.*;
import java.util.List;

/**
 * Scores a completed autonomous run for evolutionary training.
 */
public class PerformanceScorer {
    public static double scoreRun(long elapsedMs,
                                  Position finalPos,
                                  List<Checkpoint> cps) {
        double score = cps.size()*100 - (elapsedMs/100.0);
        if (!cps.isEmpty()) {
            Checkpoint last = cps.get(cps.size()-1);
            double d = finalPos.distanceTo(new Position(
                    last.x, last.y, last.heading));
            score -= d;
        }
        return score;
    }
}
