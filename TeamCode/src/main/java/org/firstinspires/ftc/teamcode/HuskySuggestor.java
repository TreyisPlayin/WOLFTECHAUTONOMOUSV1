package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

/**
 * HuskySuggestor
 *
 * Uses HuskyLens COLOR_RECOGNITION blocks to suggest how the robot
 * should move (drive/strafe) toward the next needed artifact color.
 *
 * IMPORTANT:
 *  - HuskyLens must be wired on I2C and configured as "huskylens".
 *  - HuskyLens algorithm must be COLOR_RECOGNITION and trained for your colors.
 *  - IDs here (ID_PURPLE, ID_GREEN) must match what you trained.
 */
public class HuskySuggestor {

    public static class Suggestion {
        public boolean hasTarget = false;
        public double drive = 0.0;   // forward/backward suggestion
        public double strafe = 0.0;  // left/right suggestion

        // Debug / info (optional)
        public int blockId = -1;
        public int errorX = 0;
        public int widthPx = 0;
    }

    private final HuskyLens husky;
    private final Telemetry telemetry;

    // TUNE THESE TO MATCH YOUR TRAINED COLOR IDS
    private static final int ID_PURPLE = 1;
    private static final int ID_GREEN  = 2;

    // Image is 320x240; center is (160, 120) according to FTC docs. :contentReference[oaicite:1]{index=1}
    private static final int IMG_CENTER_X = 160;

    // Tuning for movement suggestions
    private static final int CENTER_DEADBAND_PX = 20;   // how close to center counts as "centered"
    private static final int CLOSE_WIDTH_PX     = 90;   // how "large" the block is when we're close
    private static final double MAX_DRIVE       = 0.3;
    private static final double MAX_STRAFE      = 0.25;
    private static final double STRAFE_K        = 0.0035; // px → power

    public HuskySuggestor(HardwareConfig hw, Telemetry telemetry) {
        this.husky = hw.huskyLens;  // must be mapped in HardwareConfig
        this.telemetry = telemetry;
    }

    private int idForColor(ArtifactColor color) {
        if (color == null) return -1;
        switch (color) {
            case PURPLE: return ID_PURPLE;
            case GREEN:  return ID_GREEN;
            default:     return -1;
        }
    }

    /**
     * Suggest drive/strafe commands to move toward the desired color.
     *
     * This method DOES NOT touch the motors. It only returns a Suggestion.
     * Your auto state machine is responsible for blending these suggestions
     * into actual drive commands.
     */
    public Suggestion suggestForColor(ArtifactColor desiredColor) {
        Suggestion s = new Suggestion();

        if (husky == null) {
            telemetry.addLine("HuskySuggestor: huskyLens == null (check HardwareConfig & config name)");
            telemetry.update();
            return s;
        }

        int desiredId = idForColor(desiredColor);
        if (desiredId < 0) {
            telemetry.addLine("HuskySuggestor: invalid desiredColor (no ID mapping)");
            telemetry.update();
            return s;
        }

        HuskyLens.Block[] blocks = husky.blocks();
        if (blocks == null || blocks.length == 0) {
            telemetry.addLine("Husky: no blocks");
            telemetry.update();
            return s;
        }

        // Choose the block with desired ID that is "best":
        //  - matching id
        //  - largest width (closest / biggest)
        HuskyLens.Block best = null;
        for (HuskyLens.Block b : blocks) {
            if (b.id != desiredId) continue;
            if (best == null || b.width > best.width) {
                best = b;
            }
        }

        if (best == null) {
            telemetry.addData("Husky", "no block with ID=%d", desiredId);
            telemetry.addData("Husky all IDs", Arrays.toString(Arrays.stream(blocks).mapToInt(b -> b.id).toArray()));
            telemetry.update();
            return s;
        }

        int centerX = best.x;     // from FTC docs: blocks[i].x, .y are box center in pixels :contentReference[oaicite:2]{index=2}
        int width   = best.width;

        int errorX = centerX - IMG_CENTER_X;

        telemetry.addData("Husky target",
                "ID=%d x=%d w=%d errX=%d", best.id, centerX, width, errorX);

        // Horizontal suggestion (strafe)
        double strafe = 0.0;
        if (Math.abs(errorX) > CENTER_DEADBAND_PX) {
            strafe = errorX * STRAFE_K;
            if (strafe >  MAX_STRAFE) strafe =  MAX_STRAFE;
            if (strafe < -MAX_STRAFE) strafe = -MAX_STRAFE;
        }

        // Forward suggestion: move forward until block looks "big enough"
        double drive = 0.0;
        if (width < CLOSE_WIDTH_PX) {
            drive = MAX_DRIVE;
        }

        s.hasTarget = true;
        s.drive = drive;
        s.strafe = strafe;
        s.blockId = best.id;
        s.errorX = errorX;
        s.widthPx = width;

        telemetry.addData("HuskySuggestor", "drive=%.2f strafe=%.2f", drive, strafe);
        telemetry.update();

        return s;
    }
}
