package org.firstinspires.ftc.teamcode.autoassist;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

// VisionPortal / AprilTag (optional, can be disabled)
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * One-file Align+Score with:
 *  - Both-triggers gate in TeleOp (red LEDs until ready, then green; locks controls at commit)
 *  - Tiny NN to decide “ready”; training mode to label success/fail after shots
 *  - Obelisk order with a RELOAD_PURPLE state after the first (purple) score
 *  - Optional AprilTag via VisionPortal (no separate localizer class)
 *
 * USAGE (TeleOp):
 *   AlignAndScoreLite as = new AlignAndScoreLite(hw, hardwareMap, telemetry, gamepad1);
 *   as.useVision(true, "Webcam 1");        // or as.useVision(false, null) if no camera yet
 *   as.setTrainingMode(false);             // true if you want press-to-label after each shot
 *   while (opModeIsActive()) { as.tick(); if (!as.isLocked()) { /* your normal drive */ } }
        /*
        * USAGE (Auto):
        *   AlignAndScoreLite as = new AlignAndScoreLite(hw, hardwareMap, telemetry, null);
 *   as.useVision(true, "Webcam 1");
 *   as.forceStart();
 *   while (opModeIsActive()) as.tick();
 *
         * What you must wire:
        *  - drivetrain motors in HardwareConfig (leftFront/rightFront/leftRear/rightRear)
 *  - in the TODOs: startScoreFor/continueScoreFor/startReloadPurple/continueReloadPurple
 */
public class AlignAndScore {

    // ---------- Public API ----------
    public enum State { IDLE, SEEKING_TAG, ALIGNING, WAIT_READY, COMMITTED, SCORING, RELOAD_PURPLE, COMPLETE, ABORTED }
    public enum ObeliskSlot { BOTTOM, MIDDLE, TOP }

    /** Minimal hardware contract: four drive motors exposed on your HardwareConfig. */
    public static class HardwareConfig {
        public DcMotor leftFront, rightFront, leftRear, rightRear;
        // add anything else you want, e.g. lift motors, servos, etc.
    }

    public AlignAndScore(HardwareConfig hw, HardwareMap hm, Telemetry telemetry, Gamepad gp) {
        this.hw = hw;
        this.hm = hm;
        this.telemetry = telemetry;
        this.gp = gp;
        this.nn = new NeuralNet(6, 8); // [dx,dy,dh,slot,t,bias] -> 8 hidden -> 1
    }

    public void setTrainingMode(boolean enable) { this.trainingMode = enable; }

    /** Optional: enable VisionPortal + AprilTag. Provide your configured webcam name. */
    public void useVision(boolean enable, String webcamName) {
        this.useVision = enable;
        if (!enable) { shutdownVision(); return; }
        try {
            if (apriltag == null) apriltag = new AprilTagProcessor.Builder().build();
            if (portal != null) portal.close();
            VisionPortal.Builder b = new VisionPortal.Builder();
            if (webcamName != null) {
                b.setCamera(hm.get(org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName.class, webcamName));
            }
            b.addProcessor(apriltag);
            portal = b.build();
        } catch (Throwable t) {
            telemetry.addData("Vision", "Init failed: %s", t.getMessage());
            telemetry.update();
            useVision = false;
        }
    }

    /** For autonomous: start the flow immediately (no trigger gate). */
    public void forceStart() { requestStartInternal(true); }

    /** Whether the assist currently owns the robot (TeleOp should ignore other inputs). */
    public boolean isLocked() { return locked; }

    public State getState() { return state; }

    // ---------- Internals ----------
    private final HardwareConfig hw;
    private final HardwareMap hm;
    private final Telemetry telemetry;
    private final Gamepad gp; // may be null in Auto

    // Vision
    private boolean useVision = false;
    private VisionPortal portal;
    private AprilTagProcessor apriltag;

    private final ObeliskSlot[] pattern = new ObeliskSlot[]{ ObeliskSlot.BOTTOM, ObeliskSlot.MIDDLE, ObeliskSlot.TOP };
    private int patternIndex = 0;

    private final ElapsedTime timer = new ElapsedTime();
    private final NeuralNet nn;
    private double[] lastFeatures = null;

    private State state = State.IDLE;
    private boolean committed = false, locked = false, armed = false, trainingMode = false;

    // Alignment target / tunables
    private double targetX = 0.0;   // inches lateral (relative to tag)
    private double targetY = 24.0;  // inches forward
    private double targetH = 0.0;   // degrees heading error
    private double xTolIn = 1.0, yTolIn = 1.0, hTolDeg = 3.0;
    private double kP_lin = 0.03, kP_rot = 0.01, maxPow = 0.5;
    private double greenThreshold = 0.65;

    private static final double TRIG = 0.90;

    // ---------- Main loop ----------
    public void tick() {
        if (gp != null) handleTriggerGate();

        switch (state) {
            case IDLE:
            case COMPLETE:
            case ABORTED:
                break;

            case SEEKING_TAG: {
                TagPose pose = getTagPose();
                if (pose.hasPose) {
                    state = State.ALIGNING;
                } else {
                    drive(0, 0, 0.15); // slow scan
                }
            } break;

            case ALIGNING: {
                TagPose pose = getTagPose();
                if (!pose.hasPose) {
                    stopDrive();
                    state = State.SEEKING_TAG;
                    break;
                }
                double dx = pose.relXIn - targetX;
                double dy = pose.relYIn - targetY;
                double dh = wrapDeg(pose.relHeadingDeg - targetH);

                double vx = clamp(-kP_lin * dx, -maxPow, maxPow);
                double vy = clamp(-kP_lin * dy, -maxPow, maxPow);
                double om = clamp(-kP_rot * dh, -maxPow, maxPow);
                drive(vx, vy, om);

                if (Math.abs(dx) <= xTolIn && Math.abs(dy) <= yTolIn && Math.abs(dh) <= hTolDeg) {
                    stopDrive();
                    state = State.WAIT_READY;
                    timer.reset();
                }
                telemetry.addData("Align dx/dy/hdg", "%.2f / %.2f / %.1f", dx, dy, dh);
            } break;

            case WAIT_READY: {
                TagPose pose = getTagPose();
                if (!pose.hasPose) {
                    state = State.SEEKING_TAG;
                    break;
                }
                double[] feats = featuresFrom(pose, patternIndex, timer.seconds());
                lastFeatures = feats; // for training mode labeling
                double p = nn.predict(feats);
                telemetry.addData("ReadyProb", "%.2f", p);

                if (!trainingMode && p >= greenThreshold) {
                    committed = true;
                    locked = true;
                    setLed(0.0, 1.0, 0.0); // green
                    state = State.COMMITTED;
                }
            } break;

            case COMMITTED: {
                state = State.SCORING;
                timer.reset();
                startScoreFor(pattern[patternIndex]); // TODO: YOUR MECHANISMS
            } break;

            case SCORING: {
                boolean done = continueScoreFor(pattern[patternIndex]); // TODO: YOUR MECHANISMS
                if (done) {
                    if (patternIndex == 0) { // after first (purple) score
                        state = State.RELOAD_PURPLE;
                        timer.reset();
                        startReloadPurple(); // TODO: lift + pusher + flywheel
                    } else {
                        advanceOrFinish();
                    }
                }
            } break;

            case RELOAD_PURPLE: {
                boolean reloaded = continueReloadPurple(); // TODO: detect ready for next artifact
                if (reloaded) {
                    advanceOrFinish();
                }
            } break;
        }

        telemetry.addData("A&S State", state);
        telemetry.addData("Committed", committed);
        telemetry.addData("Locked", locked);
        telemetry.addData("Slot", pattern[patternIndex]);
        telemetry.update(); // addData... then update() to push to DS. :contentReference[oaicite:0]{index=0}
    }

    // ---------- Trigger/LED gate ----------
    private void handleTriggerGate() {
        boolean bothHeld = gp.left_trigger > TRIG && gp.right_trigger > TRIG;

        if (!locked) {
            if (bothHeld && !armed) requestStartInternal(false);
            if (!bothHeld && armed && !committed) cancelIfNotCommitted();

            if (state == State.SEEKING_TAG || state == State.ALIGNING || state == State.WAIT_READY) {
                setLed(1.0, 0.0, 0.0); // red while not ready
            }
        } else {
            setLed(0.0, 1.0, 0.0); // hold green while locked
        }

        // Training labels (ONLY when trainingMode is true and we’re COMPLETE)
        if (trainingMode && state == State.COMPLETE) {
            if (gp.x) recordTrainingOutcome(true);
            if (gp.b) recordTrainingOutcome(false);
        }
    }

    private void requestStartInternal(boolean force) {
        if (force || state == State.IDLE || state == State.COMPLETE || state == State.ABORTED) {
            committed = false; locked = false; armed = true;
            setLed(1.0, 0.0, 0.0); // red
            patternIndex = 0;
            state = useVision ? State.SEEKING_TAG : State.WAIT_READY; // if no vision, skip alignment
            timer.reset();
        }
    }

    private void cancelIfNotCommitted() {
        if (!committed && state != State.IDLE) {
            stopDrive();
            state = State.ABORTED;
            armed = false; locked = false;
            setLed(0.0, 0.0, 0.0);
        }
    }

    public void recordTrainingOutcome(boolean success) {
        if (!trainingMode) return;
        if (lastFeatures != null) { nn.trainLast(success); lastFeatures = null; }
        setLed(0.0, 0.0, 0.0);
        state = State.IDLE; committed = false; locked = false; armed = false;
    }

    private void advanceOrFinish() {
        if (patternIndex < pattern.length - 1) patternIndex++;
        state = State.COMPLETE; committed = false; locked = false; armed = false;
        setLed(0.0, 0.0, 0.0);
    }

    // ---------- Vision helpers ----------
    private static class TagPose { boolean hasPose; double relXIn, relYIn, relHeadingDeg; }

    private TagPose getTagPose() {
        TagPose out = new TagPose();
        if (!useVision || apriltag == null) return out;
        try {
            List<AprilTagDetection> dets = apriltag.getDetections(); // VisionPortal AprilTag detections. :contentReference[oaicite:1]{index=1}
            if (dets == null || dets.isEmpty()) return out;

            AprilTagDetection best = dets.get(0);
            for (AprilTagDetection d : dets) {
                if (d.ftcPose != null && best.ftcPose != null && d.ftcPose.range < best.ftcPose.range) best = d;
            }
            if (best.ftcPose == null) return out;

            // Map to our simple relative pose (inches + degrees). See ftcPose docs. :contentReference[oaicite:2]{index=2}
            out.hasPose = true;
            out.relXIn = best.ftcPose.x;      // left/right
            out.relYIn = best.ftcPose.y;      // forward distance
            out.relHeadingDeg = best.ftcPose.yaw; // yaw error
            return out;
        } catch (Throwable t) {
            telemetry.addData("VisionErr", t.getMessage());
            telemetry.update();
            return out;
        }
    }

    private void shutdownVision() {
        try { if (portal != null) portal.close(); } catch (Throwable ignore) {}
        portal = null; apriltag = null;
    }

    // ---------- Features for NN ----------
    private double[] featuresFrom(TagPose pose, int slotIndex, double timeAlignedSec) {
        double dx = pose.relXIn;
        double dy = pose.relYIn - targetY;
        double dh = pose.relHeadingDeg;
        double ndx = dx / 24.0, ndy = dy / 24.0, ndh = dh / 30.0;
        double slot = slotIndex;
        double t = Math.min(timeAlignedSec, 3.0) / 3.0;
        return new double[]{ ndx, ndy, ndh, slot, t, 1.0 };
    }

    // ---------- Drive (replace with your field-centric if you want) ----------
    private void drive(double vx, double vy, double om) {
        if (hw == null) return;
        hw.leftFront.setPower(vy + vx + om);
        hw.rightFront.setPower(vy - vx - om);
        hw.leftRear.setPower(vy - vx + om);
        hw.rightRear.setPower(vy + vx - om);
    }
    private void stopDrive() { drive(0,0,0); }

    // ---------- LEDs ----------
    private void setLed(double r, double g, double b) {
        if (gp == null) return;
        try {
            gp.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS); // 0..1 intensities, duration ms. :contentReference[oaicite:3]{index=3}
        } catch (Throwable ignored) {}
    }

    // ---------- Mechanism TODOs (fill these) ----------
    private void startScoreFor(ObeliskSlot slot) {
        // TODO: spin flywheel / set gates / aim for this slot. Non-blocking.
        // Example:
        // flywheel.setPower(1.0);
        // gate.setPosition(slot == ObeliskSlot.BOTTOM ? 0.25 : slot == ObeliskSlot.MIDDLE ? 0.5 : 0.75);
        // pusher.setPosition(0.0);
    }

    private boolean continueScoreFor(ObeliskSlot slot) {
        // TODO: advance your shot sequence; return true when done (sensor or timer).
        return true;
    }

    private void startReloadPurple() {
        // TODO: after first purple: lift + pusher + flywheel to prep next artifact (non-blocking).
        // lift.setTargetPosition(...); lift.setMode(RUN_TO_POSITION); lift.setPower(...);
        // pusher.setPosition(...); // cycle
        // flywheel.setPower(...);  // hold ready speed
    }

    private boolean continueReloadPurple() {
        // TODO: return true when reload is complete and ready for new artifact.
        return true;
    }

    // ---------- Math ----------
    private static double clamp(double v, double lo, double hi) { return Math.max(lo, Math.min(hi, v)); }
    private static double wrapDeg(double a) { while (a>180) a-=360; while (a<-180) a+=360; return a; }

    // ---------- Tiny NN ----------
    private static final class NeuralNet {
        private final int in, hid;
        private final double[][] w1; private final double[] b1;
        private final double[] w2; private double b2;
        private final double lr = 0.05;
        private double[] lastX, lastH; private double lastY;

        NeuralNet(int input, int hidden) {
            this.in = input; this.hid = hidden;
            this.w1 = new double[hidden][input]; this.b1 = new double[hidden];
            this.w2 = new double[hidden]; this.b2 = 0.0;
        }
        double predict(double[] x) {
            lastX = x.clone();
            double[] h = new double[hid];
            for (int i = 0; i < hid; i++) {
                double s = b1[i];
                for (int j = 0; j < in; j++) s += w1[i][j] * x[j];
                h[i] = Math.max(0.0, s); // ReLU
            }
            lastH = h;
            double z = b2;
            for (int i = 0; i < hid; i++) z += w2[i] * h[i];
            lastY = 1.0 / (1.0 + Math.exp(-z)); // sigmoid
            return lastY;
        }
        void trainLast(boolean success) {
            if (lastX == null || lastH == null) return;
            double target = success ? 1.0 : 0.0;
            double err = lastY - target; // BCE derivative wrt z
            for (int i = 0; i < hid; i++) w2[i] -= lr * err * lastH[i];
            b2 -= lr * err;
            for (int i = 0; i < hid; i++) {
                double grad = err * w2[i];
                if (lastH[i] <= 0.0) grad = 0.0; // ReLU derivative
                for (int j = 0; j < in; j++) w1[i][j] -= lr * grad * lastX[j];
                b1[i] -= lr * grad;
            }
            lastX = null; lastH = null;
        }
    }
}
