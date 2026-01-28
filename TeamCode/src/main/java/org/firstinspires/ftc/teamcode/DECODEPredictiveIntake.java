package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.io.File;
import java.io.FileWriter;
import java.io.PrintWriter;
import java.util.LinkedList;

/**
 * DECODE Predictive Intake System (2025-2026)
 * Features: Spatial Bayesian Confidence, Internal Data Logging, 
 * Voltage Scaling, and Motif-Bypass Logic.
 */
public class DECODEPredictiveIntake {
    // --- MOTIF CONFIGURATION ---
    public static char[][] MOTIF_MAP = {{'G','P','P'}, {'P','G','P'}, {'P','P','G'}};
    public boolean followMotif = true;
    public int motifStep = 0;

    // --- TUNABLE CONSTANTS (Adjust via Lab/Tuner) ---
    public static double PUSH_L = 0.2, PUSH_R = 0.8, NEUTRAL = 0.5;
    public static double STORAGE_EXTEND = 1.0, STORAGE_RETRACT = 0.0;
    public static float GREEN_HUE = 140f, PURPLE_HUE = 235f, HUE_TOLERANCE = 30f;
    public static double CONFIDENCE_THRESHOLD = 0.85; // 85% Certainty required
    public static double FRONT_TRIGGER_DIST = 3.5;    // cm
    public static double MISS_TIMEOUT = 10.0;         // seconds

    // --- HARDWARE ---
    public CRServo intake, augerL, augerR;
    public Servo lrPusher, lPush, rPush;
    public ColorSensor funnelL, funnelR, frontV3, backV3;
    public DigitalChannel siloL, siloR;
    private VoltageSensor battery;

    // --- SPATIAL BAYESIAN STATE ---
    private LinkedList<Character> confidenceBuffer = new LinkedList<>();
    private boolean ballPending = false;
    private ElapsedTime pendingTimer = new ElapsedTime();
    private String logPath = "/sdcard/FIRST/data/intake_telemetry.csv";

    public DECODEPredictiveIntake(HardwareMap hw) {
        // Init Motors/Servos
        intake = hw.get(CRServo.class, "intake servo");
        augerL = hw.get(CRServo.class, "leftAuger");
        augerR = hw.get(CRServo.class, "rightAuger");
        lrPusher = hw.get(Servo.class, "LR pusher");
        lPush = hw.get(Servo.class, "Lpusher");
        rPush = hw.get(Servo.class, "Rpusher");

        // Init Sensors
        funnelL = hw.get(ColorSensor.class, "funnelLeft");
        funnelR = hw.get(ColorSensor.class, "funnelRight");
        frontV3 = hw.get(ColorSensor.class, "centerFront");
        backV3 = hw.get(ColorSensor.class, "centerBack");
        siloL = hw.get(DigitalChannel.class, "distL");
        siloR = hw.get(DigitalChannel.class, "distR");
        battery = hw.voltageSensor.iterator().next();

        siloL.setMode(DigitalChannel.Mode.INPUT);
        siloR.setMode(DigitalChannel.Mode.INPUT);
    }

    /**
     * Main control loop for the intake.
     * @param tagId The ID from the Obelisk AprilTag (21, 22, or 23).
     * @param intakeActive Should the intake be spinning?
     */
    public void update(int tagId, boolean intakeActive) {
        double vScale = 12.0 / battery.getVoltage();
        char target = (tagId >= 21 && tagId <= 23 && motifStep < 3) ? MOTIF_MAP[tagId-21][motifStep] : 'N';

        // 1. Run Spatial Bayesian Inference (Always active)
        char detectedColor = runAIInference();

        // 2. Logic Selection
        if (!followMotif) {
            // FAST BYPASS MODE: Max speed, alternate silos
            intake.setPower(intakeActive ? 1.0 : 0);
            lrPusher.setPosition(motifStep % 2 == 0 ? PUSH_L : PUSH_R);
        } else {
            // MOTIF SORTING MODE
            intake.setPower(intakeActive ? 0.85 * vScale : 0);

            // Only make a decision if we aren't already processing a ball
            if (detectedColor != 'N' && !ballPending) {
                if (detectedColor == target) {
                    ballPending = true;
                    pendingTimer.reset();
                    // Choose silo based on current step
                    lrPusher.setPosition(motifStep % 2 == 0 ? PUSH_L : PUSH_R);
                } else {
                    // Wrong color for motif: Let it pass out the back
                    lrPusher.setPosition(NEUTRAL);
                }
            }
        }

        // 3. Watchdog Reset (Clears stuck state)
        if (ballPending && pendingTimer.seconds() > MISS_TIMEOUT) {
            ballPending = false;
            lrPusher.setPosition(NEUTRAL);
        }

        // 4. Storage & Auger Control
        updateStorage(vScale);
    }

    private void updateStorage(double scale) {
        // If Silo sensors triggered (Active Low assumed)
        if (!siloL.getState()) {
            lPush.setPosition(STORAGE_EXTEND);
            augerL.setPower(0.7 * scale);
            if (ballPending) { motifStep++; ballPending = false; }
        } else {
            lPush.setPosition(STORAGE_RETRACT);
            augerL.setPower(0);
        }

        if (!siloR.getState()) {
            rPush.setPosition(STORAGE_EXTEND);
            augerR.setPower(0.7 * scale);
            if (ballPending) { motifStep++; ballPending = false; }
        } else {
            rPush.setPosition(STORAGE_RETRACT);
            augerR.setPower(0);
        }
    }

    private char runAIInference() {
        float[] hsv = new float[3];
        Color.RGBToHSV(frontV3.red(), frontV3.green(), frontV3.blue(), hsv);

        char currentSample = 'N';
        if (Math.abs(hsv[0] - GREEN_HUE) < HUE_TOLERANCE) currentSample = 'G';
        else if (Math.abs(hsv[0] - PURPLE_HUE) < HUE_TOLERANCE) currentSample = 'P';

        // Slide the window
        confidenceBuffer.add(currentSample);
        if (confidenceBuffer.size() > 10) confidenceBuffer.removeFirst();

        // Calculate Spatial Probability
        int gCount = 0, pCount = 0;
        for (char c : confidenceBuffer) {
            if (c == 'G') gCount++;
            if (c == 'P') pCount++;
        }

        if ((double)gCount / 10.0 > CONFIDENCE_THRESHOLD) return 'G';
        if ((double)pCount / 10.0 > CONFIDENCE_THRESHOLD) return 'P';
        return 'N';
    }

    public void logCurrentState() {
        try {
            File dir = new File("/sdcard/FIRST/data/");
            if (!dir.exists()) dir.mkdirs();
            PrintWriter out = new PrintWriter(new FileWriter(logPath, true));
            out.printf("%d, %b, %d, %s\n",
                    System.currentTimeMillis(), ballPending, motifStep, followMotif ? "MOTIF" : "FAST");
            out.close();
        } catch (Exception e) { /* Log silenty */ }
    }
}