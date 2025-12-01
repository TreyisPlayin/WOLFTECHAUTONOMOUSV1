package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "PatternAuto_HuskySuggest", group = "Autonomous")
public class PatternAuto extends LinearOpMode {

    private HardwareConfig robot;
    private LocalizationManager loc;    // if you already have it
    private PickupManager pickup;       // your existing pickup manager
    private ShooterManager shooter;     // your existing shooter manager
    private HuskySuggestor huskySuggestor;

    // Simple motif: PURPLE, GREEN, PURPLE
    private final ArtifactColor[] motif = new ArtifactColor[]{
            ArtifactColor.PURPLE,
            ArtifactColor.GREEN,
            ArtifactColor.PURPLE
    };
    private int motifIndex = 0;

    // State machine for auto
    private enum AutoState {
        DRIVE_TO_PRELOAD_SPOT,
        SCORE_PRELOAD,
        LOOKING_FOR_PICKUP,
        PICKING_UP,
        DRIVE_TO_GOAL,
        SCORE_MOTIF,
        DONE
    }

    private AutoState state = AutoState.DRIVE_TO_PRELOAD_SPOT;

    @Override
    public void runOpMode() {
        // Hardware init
        robot = new HardwareConfig(this);
        robot.init(hardwareMap, telemetry);

        // Localization (pinpoint + optional vision portal, if you created that class)
        loc = new LocalizationManager(hardwareMap, robot.pinpoint);
        loc.start();

        // Managers (you already have these classes)
        pickup = new PickupManager(robot, telemetry);
        shooter = new ShooterManager(robot, telemetry);
        huskySuggestor = new HuskySuggestor(robot, telemetry);

        telemetry.addLine("PatternAuto_HuskySuggest: Init complete");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Example: immediately score whatever preload you have, then go into search.
        state = AutoState.SCORE_PRELOAD;

        while (opModeIsActive() && state != AutoState.DONE) {
            loc.update(); // keep pose fresh, even if we don't use it heavily

            switch (state) {
                case DRIVE_TO_PRELOAD_SPOT:
                    doDriveToPreloadSpot();
                    state = AutoState.SCORE_PRELOAD;
                    break;

                case SCORE_PRELOAD:
                    doScorePreload();
                    state = AutoState.LOOKING_FOR_PICKUP;
                    break;

                case LOOKING_FOR_PICKUP:
                    if (doLookingForPickup()) {
                        // once we think we're close enough, move to intake phase
                        state = AutoState.PICKING_UP;
                    }
                    break;

                case PICKING_UP:
                    if (doPickingUp()) {
                        state = AutoState.DRIVE_TO_GOAL;
                    }
                    break;

                case DRIVE_TO_GOAL:
                    doDriveToGoal();
                    state = AutoState.SCORE_MOTIF;
                    break;

                case SCORE_MOTIF:
                    doScoreMotif();
                    state = AutoState.DONE;
                    break;

                case DONE:
                default:
                    stopDrive();
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("MotifIndex", motifIndex);
            telemetry.update();
        }

        stopDrive();
    }

    // -------------------------
    // STATE IMPLEMENTATIONS
    // -------------------------

    private void doDriveToPreloadSpot() {
        // Simple timed move as placeholder; you can replace this with FieldNav or
        // more precise logic.
        driveMecanum(0.4, 0, 0);
        sleep(800);
        stopDrive();
    }

    private void doScorePreload() {
        // Spin up shooter and fire what you already have loaded.
        double targetRPM = 3200.0; // you will tune this
        shooter.setTargetRPM(targetRPM);
        shooter.waitUntilReady();
        shooter.setLedReady(true);

        // Fire your preloaded motif. You can make this more advanced later.
        for (ArtifactColor c : motif) {
            // trivial: choose channel based on color
            int channel = (c == ArtifactColor.PURPLE) ? 0 : 1;
            shooter.fireOneFromChannel(channel);
            sleep(200);
        }

        shooter.stopAll();
        shooter.setLedReady(false);
    }

    /**
     * LOOKING_FOR_PICKUP state:
     *  - Ask pickup manager what color we need next.
     *  - Ask HuskySuggestor for drive/strafe suggestions toward that color.
     *  - Blend suggestions into mecanum drive.
     *  - Return true if we think we're close enough that intake should take over.
     */
    private boolean doLookingForPickup() {
        // Ask pickup manager what we need next; if you don't have this method, you can
        // replace it with motif[motifIndex] or something similar.
        ArtifactColor needed = getNeededColor();

        HuskySuggestor.Suggestion s = huskySuggestor.suggestForColor(needed);

        double drive = 0.0;
        double strafe = 0.0;
        double turn = 0.0;

        if (s.hasTarget) {
            // Use the suggestions
            drive = s.drive;
            strafe = s.strafe;
        } else {
            // Simple fallback search behavior when we don't see the color:
            // slowly rotate to scan the field.
            turn = 0.2;
        }

        driveMecanum(drive, strafe, turn);

        // Heuristic: if Husky sees the color and block is "big enough", we are close.
        // This is the same logic HuskySuggestor used to stop suggesting drive.
        boolean alignedAndClose = s.hasTarget && Math.abs(s.errorX) <= 20 && s.widthPx >= 90;

        // You can also add a timeout here so it doesn't search forever.
        return alignedAndClose;
    }

    /**
     * PICKING_UP state:
     *  - Let PickupManager do its normal classification/intake using bottom sensors.
     *  - Once we think we've successfully loaded the needed ball, advance motifIndex.
     *  - Return true when pickup is done (ready to drive back to goal).
     *
     * You WILL need to adjust the method calls below to match your actual PickupManager API.
     */
    private boolean doPickingUp() {
        ArtifactColor needed = getNeededColor();

        // Let your existing manager run its intake logic; in your real code this might
        // use the bottom color sensors, distance sensors, augers, and gates.
        pickup.update(); // adjust to your actual method name/signature

        // Example exit condition (you will replace with whatever your class exposes):
        boolean gotBall = pickup.hasBallOfColor(needed); // <- adjust this to your API

        if (gotBall) {
            motifIndex++;
            if (motifIndex >= motif.length) {
                motifIndex = motif.length - 1; // clamp
            }
            return true;
        }
        return false;
    }

    private void doDriveToGoal() {
        // Basic timed drive back toward the goal. You can replace this with more
        // advanced navigation using LocalizationManager.
        driveMecanum(-0.4, 0, 0);
        sleep(1000);
        stopDrive();
    }

    private void doScoreMotif() {
        // Re-score motif with whatever you collected.
        // For now, just reuse the same simple logic as preload.
        double targetRPM = 3200.0;
        shooter.setTargetRPM(targetRPM);
        shooter.waitUntilReady();
        shooter.setLedReady(true);

        for (ArtifactColor c : motif) {
            int channel = (c == ArtifactColor.PURPLE) ? 0 : 1;
            shooter.fireOneFromChannel(channel);
            sleep(200);
        }

        shooter.stopAll();
        shooter.setLedReady(false);
    }

    // -------------------------
    // SUPPORT
    // -------------------------

    private ArtifactColor getNeededColor() {
        // If your PickupManager already tracks the next needed color, use that:
        // return pickup.getNextNeededColor();
        //
        // For now, use the motif index as a simple stand-in:
        if (motifIndex < 0 || motifIndex >= motif.length) {
            return ArtifactColor.PURPLE;
        }
        return motif[motifIndex];
    }

    private void driveMecanum(double drive, double strafe, double turn) {
        double fl = drive + strafe + turn;
        double rl = drive - strafe + turn;
        double fr = drive - strafe - turn;
        double rr = drive + strafe - turn;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(rl), Math.max(Math.abs(fr), Math.abs(rr)))));

        fl /= max;
        rl /= max;
        fr /= max;
        rr /= max;

        robot.leftFront.setPower(fl);
        robot.leftRear.setPower(rl);
        robot.rightFront.setPower(fr);
        robot.rightRear.setPower(rr);
    }

    private void stopDrive() {
        driveMecanum(0, 0, 0);
    }
}
