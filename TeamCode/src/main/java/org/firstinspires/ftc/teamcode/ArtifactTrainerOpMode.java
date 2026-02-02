package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import android.graphics.Color;
import java.io.File;
import java.io.FileOutputStream;

@TeleOp(name="Artifact Trainer - Ranges + Probe")
public class ArtifactTrainerOpMode extends OpMode {

    private static final String LEFT_V2 = "leftColor";
    private static final String RIGHT_V2 = "rightColor";
    private static final String SERVO_COLOR = "servoColor"; // also distance

    private ColorSensor leftV2, rightV2, servoColor;
    private DistanceSensor servoDistance;

    private V2DataCollector v2collector;
    private ServoSensorTrainer servoTrainer;

    // modes / UI
    private boolean servoTrainMode = false;
    private boolean probeMode = false;

    private boolean prevY = false, prevDUp = false, prevDDown = false, prevX = false, prevA = false, prevB = false, prevStart = false;
    private boolean probeResetPending = false;

    @Override
    public void init() {
        leftV2 = hardwareMap.get(ColorSensor.class, LEFT_V2);
        rightV2 = hardwareMap.get(ColorSensor.class, RIGHT_V2);
        servoColor = hardwareMap.get(ColorSensor.class, SERVO_COLOR);
        servoDistance = hardwareMap.get(DistanceSensor.class, SERVO_COLOR);

        v2collector = new V2DataCollector(leftV2, rightV2, 0.12);
        servoTrainer = new ServoSensorTrainer(servoColor, servoDistance);

        telemetry.addLine("Init done. Y=toggle servo train. DPAD_RIGHT=toggle probe. DPadUp/Down select V2 label.");
        telemetry.update();
    }

    @Override
    public void loop() {
        long now = System.currentTimeMillis();

        // button toggles
        boolean y = gamepad1.y;
        if (y && !prevY) servoTrainMode = !servoTrainMode;
        prevY = y;

        // toggle probe mode on DPad right rising edge
        boolean dRight = gamepad1.dpad_right;
        if (dRight && !prevDUp) probeMode = !probeMode;
        prevDUp = dRight;

        // choose V2 label with DPad up/down
        if (gamepad1.dpad_up) v2collector.setCurrentLabel(V2DataCollector.Label.GREEN);
        if (gamepad1.dpad_down) v2collector.setCurrentLabel(V2DataCollector.Label.PURPLE);

        // hold LB to record labeled V2 samples
        v2collector.setRecording(gamepad1.left_bumper);
        if (gamepad1.left_bumper) v2collector.tryRecordSample();

        // servo train mode controls
        boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, start = gamepad1.start;

        if (servoTrainMode) {
            telemetry.addLine("MODE: SERVO TRAIN");
            telemetry.addData("Green samples", servoTrainer.greenCount());
            telemetry.addData("Purple samples", servoTrainer.purpleCount());
            telemetry.addLine("A=addGreen, B=addPurple, X=compute+save, START=clear");

            if (a && !prevA) servoTrainer.addGreenSample();
            if (b && !prevB) servoTrainer.addPurpleSample();
            if (x && !prevX) {
                // compute ranges and save to file (min/max)
                if (servoTrainer.hasEnoughSamples()) {
                    String out = buildServoRangeText();
                    saveTextFile("/sdcard/FIRST/servo_ranges.txt", out);
                    telemetry.addLine("Computed servo ranges saved to /sdcard/FIRST/servo_ranges.txt");
                    telemetry.addData("servo ranges", out);
                } else telemetry.addLine("Not enough servo samples (need >=3 each).");
            }
            if (start && !prevStart) {
                servoTrainer.reset();
                telemetry.addLine("Servo trainer cleared");
            }
        } else if (probeMode) {
            // Probe Mode: show live classification using trained ranges; pressing A resets detection
            telemetry.addLine("MODE: PROBE (live classification)");
            // read sensors
            int lR = leftV2.red(), lG = leftV2.green(), lB = leftV2.blue();
            int rR = rightV2.red(), rG = rightV2.green(), rB = rightV2.blue();
            int sR = servoColor.red(), sG = servoColor.green(), sB = servoColor.blue();
            double sDist = servoDistance.getDistance(com.qualcomm.robotcore.hardware.DistanceSensor.DistanceUnit.CM);

            // compute hue+sat+val
            float[] hsv = new float[3];
            Color.RGBToHSV(clamp(lR), clamp(lG), clamp(lB), hsv);
            float lHue = hsv[0], lSat = hsv[1], lVal = hsv[2];
            Color.RGBToHSV(clamp(rR), clamp(rG), clamp(rB), hsv);
            float rHue = hsv[0], rSat = hsv[1], rVal = hsv[2];
            Color.RGBToHSV(clamp(sR), clamp(sG), clamp(sB), hsv);
            float sHue = hsv[0], sSat = hsv[1], sVal = hsv[2];

            // compute confidence using V2 ranges if available; else use servo ranges if available
            V2DataCollector.Range3 v2g = v2collector.getGreenRange();
            V2DataCollector.Range3 v2p = v2collector.getPurpleRange();

            float v2ConfidenceGreen = 0f, v2ConfidencePurple = 0f;
            if (v2g != null && v2p != null) {
                v2ConfidenceGreen = sensorMatchConfidence(lHue, lSat, lVal, v2g) * 0.5f
                        + sensorMatchConfidence(rHue, rSat, rVal, v2g) * 0.5f;
                v2ConfidencePurple = sensorMatchConfidence(lHue, lSat, lVal, v2p) * 0.5f
                        + sensorMatchConfidence(rHue, rSat, rVal, v2p) * 0.5f;
            }

            float servoConfidenceGreen = 0f, servoConfidencePurple = 0f;
            if (servoTrainer.greenCount() >= 1 && servoTrainer.purpleCount() >= 1) {
                // check servo hue ranges
                float gMinH = servoTrainer.greenHueMin(), gMaxH = servoTrainer.greenHueMax();
                float pMinH = servoTrainer.purpleHueMin(), pMaxH = servoTrainer.purpleHueMax();
                float gMinD = servoTrainer.greenDistMinCm(), gMaxD = servoTrainer.greenDistMaxCm();
                float pMinD = servoTrainer.purpleDistMinCm(), pMaxD = servoTrainer.purpleDistMaxCm();

                servoConfidenceGreen = hueDistanceScore(sHue, gMinH, gMaxH) * 0.7f + distanceScore((float)sDist, gMinD, gMaxD) * 0.3f;
                servoConfidencePurple = hueDistanceScore(sHue, pMinH, pMaxH) * 0.7f + distanceScore((float)sDist, pMinD, pMaxD) * 0.3f;
            }

            // combine confidences: prefer V2 if available, else servo
            float combinedGreen = (v2g != null) ? v2ConfidenceGreen : servoConfidenceGreen;
            float combinedPurple = (v2p != null) ? v2ConfidencePurple : servoConfidencePurple;

            String detected;
            float conf;
            if (combinedGreen > combinedPurple) { detected = "GREEN"; conf = combinedGreen; }
            else { detected = "PURPLE"; conf = combinedPurple; }

            telemetry.addData("Probe Detected", "%s (%.2f)", detected, conf);
            telemetry.addData("V2 conf G/P", "%.2f/%.2f", v2ConfidenceGreen, v2ConfidencePurple);
            telemetry.addData("Servo conf G/P", "%.2f/%.2f (hue,dist)", servoConfidenceGreen, servoConfidencePurple);
            telemetry.addData("Servo dist cm", "%.2f", sDist);
            telemetry.addLine("Press A to reset probe detection (clear any transient state)");

            // probe reset on A (rising edge)
            if (a && !prevA) {
                probeResetPending = true;
                // clear any transient internal flags (none persistent here) — just report
                telemetry.addLine("Probe detection reset");
            }
        } else {
            // predictor/training mode (no predictor usage per your request)
            telemetry.addLine("MODE: V2 collection / compute");
            telemetry.addData("V2 label (DPad Up/Down)", v2collector.getCurrentLabel().name());
            telemetry.addData("Hold LB to record V2 samples", v2collector.isRecording());
            telemetry.addData("V2 sample counts G/P", "%d / %d", v2collector.greenSampleCount(), v2collector.purpleSampleCount());
            telemetry.addLine("DPadUp=GREEN label, DPadDown=PURPLE label, Hold LB to record");
            telemetry.addLine("X=compute V2 ranges+save; START=save CSV");
            if (x && !prevX) {
                // compute V2 ranges and write to file
                V2DataCollector.Range3 g = v2collector.getGreenRange();
                V2DataCollector.Range3 p = v2collector.getPurpleRange();
                if (g == null || p == null) telemetry.addLine("Not enough samples to compute V2 ranges (need >=3 per color).");
                else {
                    String out = buildV2RangeText(g, p);
                    saveTextFile("/sdcard/FIRST/v2_ranges.txt", out);
                    telemetry.addLine("V2 ranges computed and saved to /sdcard/FIRST/v2_ranges.txt");
                    telemetry.addData("V2 ranges", out);
                }
            }
            if (start && !prevStart) {
                String csv = v2collector.getCsv();
                saveTextFile("/sdcard/FIRST/v2_samples.csv", csv);
                telemetry.addLine("V2 CSV saved to /sdcard/FIRST/v2_samples.csv");
            }
        }

        // update prevs
        prevA = a; prevB = b; prevX = x; prevStart = start;

        telemetry.update();
    }

    private float sensorMatchConfidence(float hue, float sat, float val, V2DataCollector.Range3 r) {
        // 1. if hue inside range => 1, else linearly decay by angle distance (max 60 deg)
        float hueScore = 0f;
        if (hue >= r.minHue && hue <= r.maxHue) hueScore = 1f;
        else {
            float diff = Math.min(Math.abs(hue - r.minHue), Math.abs(hue - r.maxHue));
            hueScore = Math.max(0f, 1f - diff / 60f);
        }
        // 2. sat/val inside range -> 1 else linear drop
        float satScore = (sat >= r.minSat && sat <= r.maxSat) ? 1f : Math.max(0f, 1f - Math.abs(sat - ((r.minSat + r.maxSat)/2f)) / 0.5f);
        float valScore = (val >= r.minVal && val <= r.maxVal) ? 1f : Math.max(0f, 1f - Math.abs(val - ((r.minVal + r.maxVal)/2f)) / 0.5f);

        return (hueScore * 0.6f + satScore * 0.2f + valScore * 0.2f);
    }

    private float hueDistanceScore(float hue, float minH, float maxH) {
        if (Float.isNaN(minH) || Float.isNaN(maxH)) return 0f;
        if (hue >= minH && hue <= maxH) return 1f;
        float diff = Math.min(Math.abs(hue - minH), Math.abs(hue - maxH));
        return Math.max(0f, 1f - diff / 60f);
    }
    private float distanceScore(float d, float minD, float maxD) {
        if (Float.isNaN(minD) || Float.isNaN(maxD)) return 0f;
        if (d >= minD && d <= maxD) return 1f;
        float diff = Math.min(Math.abs(d - minD), Math.abs(d - maxD));
        // scale: 0..10 cm tolerance
        return Math.max(0f, 1f - diff / 10f);
    }

    private String buildV2RangeText(V2DataCollector.Range3 g, V2DataCollector.Range3 p) {
        return String.format("GREEN hue:[%.1f,%.1f] sat:[%.3f,%.3f] val:[%.3f,%.3f]\nPURPLE hue:[%.1f,%.1f] sat:[%.3f,%.3f] val:[%.3f,%.3f]\n",
                g.minHue, g.maxHue, g.minSat, g.maxSat, g.minVal, g.maxVal,
                p.minHue, p.maxHue, p.minSat, p.maxSat, p.minVal, p.maxVal);
    }

    private String buildServoRangeText() {
        return String.format("GREEN hue:[%.1f,%.1f] dist:[%.2f,%.2f] cm\nPURPLE hue:[%.1f,%.1f] dist:[%.2f,%.2f] cm\nMID_HUE:%.2f MID_DIST:%.2f\n",
                servoTrainer.greenHueMin(), servoTrainer.greenHueMax(), servoTrainer.greenDistMinCm(), servoTrainer.greenDistMaxCm(),
                servoTrainer.purpleHueMin(), servoTrainer.purpleHueMax(), servoTrainer.purpleDistMinCm(), servoTrainer.purpleDistMaxCm(),
                servoTrainer.computedHueMidpoint(), servoTrainer.computedDistanceThresholdCm());
    }

    // simple file writer
    private void saveTextFile(String path, String text) {
        try {
            File f = new File(path);
            File dir = f.getParentFile();
            if (dir != null && !dir.exists()) dir.mkdirs();
            FileOutputStream fos = new FileOutputStream(f);
            fos.write(text.getBytes());
            fos.flush();
            fos.close();
        } catch (Exception e) {
            telemetry.addData("FileSaveErr", e.getMessage());
        }
    }

    private int clamp(int v) { if (v < 0) return 0; if (v > 255) return 255; return v; }
}
