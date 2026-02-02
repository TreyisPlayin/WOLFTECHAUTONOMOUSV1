package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import java.util.ArrayList;
import java.util.List;

/**
 * V2 data recorder with labeled samples (GREEN/PURPLE).
 * Recording only adds a labeled sample when recording==true and sample strength > minStrength.
 * After collection call computeRanges() to get min/max hue/sat/val ranges per color.
 */
public class V2DataCollector {

    public enum Label { GREEN, PURPLE }

    public static class Range3 {
        public final float minHue, maxHue;
        public final float minSat, maxSat;
        public final float minVal, maxVal;
        public Range3(float minHue, float maxHue,
                      float minSat, float maxSat,
                      float minVal, float maxVal) {
            this.minHue = minHue; this.maxHue = maxHue;
            this.minSat = minSat; this.maxSat = maxSat;
            this.minVal = minVal; this.maxVal = maxVal;
        }
    }

    private final ColorSensor leftSensor;
    private final ColorSensor rightSensor;
    private final double minStrengthToRecord;
    private final List<Float> greenHues = new ArrayList<>();
    private final List<Float> greenSats = new ArrayList<>();
    private final List<Float> greenVals = new ArrayList<>();
    private final List<Float> purpleHues = new ArrayList<>();
    private final List<Float> purpleSats = new ArrayList<>();
    private final List<Float> purpleVals = new ArrayList<>();
    private boolean recording = false;
    private Label currentLabel = Label.GREEN;

    public V2DataCollector(ColorSensor left, ColorSensor right, double minStrengthToRecord) {
        this.leftSensor = left;
        this.rightSensor = right;
        this.minStrengthToRecord = minStrengthToRecord;
    }

    public void setRecording(boolean on) { this.recording = on; }
    public boolean isRecording() { return recording; }
    public void setCurrentLabel(Label l) { this.currentLabel = l; }
    public Label getCurrentLabel() { return currentLabel; }

    // call in loop while recording is true (holds button)
    public void tryRecordSample() {
        if (!recording) return;

        int lR = leftSensor.red(), lG = leftSensor.green(), lB = leftSensor.blue();
        int rR = rightSensor.red(), rG = rightSensor.green(), rB = rightSensor.blue();

        float[] hsv = new float[3];
        Color.RGBToHSV(clamp(lR), clamp(lG), clamp(lB), hsv);
        float lHue = hsv[0], lSat = hsv[1], lVal = hsv[2];
        double lStr = lSat * lVal;

        Color.RGBToHSV(clamp(rR), clamp(rG), clamp(rB), hsv);
        float rHue = hsv[0], rSat = hsv[1], rVal = hsv[2];
        double rStr = rSat * rVal;

        if (lStr < minStrengthToRecord && rStr < minStrengthToRecord) return;

        // push both sensors' HSV into the label lists (so both sides contribute)
        if (currentLabel == Label.GREEN) {
            greenHues.add(lHue); greenHues.add(rHue);
            greenSats.add(lSat); greenSats.add(rSat);
            greenVals.add(lVal); greenVals.add(rVal);
        } else {
            purpleHues.add(lHue); purpleHues.add(rHue);
            purpleSats.add(lSat); purpleSats.add(rSat);
            purpleVals.add(lVal); purpleVals.add(rVal);
        }
    }

    public int greenSampleCount() { return greenHues.size(); }
    public int purpleSampleCount() { return purpleHues.size(); }

    // compute per-color min/max ranges (hue, sat, val). Return null if not enough data.
    public Range3 computeRanges() {
        if (greenHues.size() < 3 || purpleHues.size() < 3) return null;
        // compute min/max for green
        float gMinH = minFloatList(greenHues), gMaxH = maxFloatList(greenHues);
        float gMinS = minFloatList(greenSats), gMaxS = maxFloatList(greenSats);
        float gMinV = minFloatList(greenVals), gMaxV = maxFloatList(greenVals);

        // compute min/max for purple
        float pMinH = minFloatList(purpleHues), pMaxH = maxFloatList(purpleHues);
        float pMinS = minFloatList(purpleSats), pMaxS = maxFloatList(purpleSats);
        float pMinV = minFloatList(purpleVals), pMaxV = maxFloatList(purpleVals);

        // Return combined range object representing "both colors' ranges" via storing GREEN range in minHue..maxHue
        // and encoding purple range into separate fields by using a compound approach is awkward;
        // instead expose getters below for each color separately. Here we return green-range as example.
        return new Range3(gMinH, gMaxH, gMinS, gMaxS, gMinV, gMaxV);
    }

    // accessors for both color ranges
    public Range3 getGreenRange() {
        if (greenHues.isEmpty()) return null;
        return new Range3(minFloatList(greenHues), maxFloatList(greenHues),
                minFloatList(greenSats), maxFloatList(greenSats),
                minFloatList(greenVals), maxFloatList(greenVals));
    }
    public Range3 getPurpleRange() {
        if (purpleHues.isEmpty()) return null;
        return new Range3(minFloatList(purpleHues), maxFloatList(purpleHues),
                minFloatList(purpleSats), maxFloatList(purpleSats),
                minFloatList(purpleVals), maxFloatList(purpleVals));
    }

    // export CSV of stored HSV per sample (one row per added pair)
    public String getCsv() {
        StringBuilder sb = new StringBuilder();
        sb.append("label,leftHue,leftSat,leftVal,rightHue,rightSat,rightVal\n");
        // We do not store paired per-sample grouping; reconstruct rows by pairs in lists.
        // Since we added both sensors together, iterate by step=2.
        int maxLen = Math.max(greenHues.size(), purpleHues.size());
        // Simpler: produce two CSV sections: green then purple, sensor by sensor.
        sb.append("# Green samples (each sensor separately)\n");
        for (int i = 0; i < greenHues.size(); i += 2) {
            int j = i + 1;
            if (j >= greenHues.size()) break;
            sb.append("GREEN,");
            sb.append(String.format("%.2f,%.3f,%.3f,", greenHues.get(i), greenSats.get(i), greenVals.get(i)));
            sb.append(String.format("%.2f,%.3f,%.3f\n", greenHues.get(j), greenSats.get(j), greenVals.get(j)));
        }
        sb.append("# Purple samples (each sensor separately)\n");
        for (int i = 0; i < purpleHues.size(); i += 2) {
            int j = i + 1;
            if (j >= purpleHues.size()) break;
            sb.append("PURPLE,");
            sb.append(String.format("%.2f,%.3f,%.3f,", purpleHues.get(i), purpleSats.get(i), purpleVals.get(i)));
            sb.append(String.format("%.2f,%.3f,%.3f\n", purpleHues.get(j), purpleSats.get(j), purpleVals.get(j)));
        }
        return sb.toString();
    }

    public void clear() {
        greenHues.clear(); greenSats.clear(); greenVals.clear();
        purpleHues.clear(); purpleSats.clear(); purpleVals.clear();
    }

    // helpers
    private float minFloatList(List<Float> list) {
        float m = Float.POSITIVE_INFINITY;
        for (float v : list) if (v < m) m = v;
        return m;
    }
    private float maxFloatList(List<Float> list) {
        float m = Float.NEGATIVE_INFINITY;
        for (float v : list) if (v > m) m = v;
        return m;
    }
    private int clamp(int v) { if (v < 0) return 0; if (v > 255) return 255; return v; }
}
