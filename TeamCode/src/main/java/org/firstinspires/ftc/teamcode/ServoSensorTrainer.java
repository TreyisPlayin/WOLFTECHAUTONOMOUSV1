package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.ArrayList;
import java.util.List;

/**
 * Servo trainer collects labeled samples of both hue (from the V3 color sensor)
 * and distance (from the V3 distance sensor interface) for GREEN and PURPLE artifacts.
 * Computes min/max ranges for hue and distance per color and exposes them for training output.
 */
public class ServoSensorTrainer {

    private final ColorSensor servoColor;
    private final DistanceSensor servoDistance;

    private final List<Float> greenHues = new ArrayList<>();
    private final List<Float> purpleHues = new ArrayList<>();
    private final List<Float> greenDists = new ArrayList<>();
    private final List<Float> purpleDists = new ArrayList<>();

    public ServoSensorTrainer(ColorSensor servoColor, DistanceSensor servoDistance) {
        this.servoColor = servoColor;
        this.servoDistance = servoDistance;
    }

    public void addGreenSample() {
        float[] hsv = new float[3];
        Color.RGBToHSV(clamp(servoColor.red()), clamp(servoColor.green()), clamp(servoColor.blue()), hsv);
        greenHues.add(hsv[0]);
        greenDists.add((float)servoDistance.getDistance(DistanceUnit.CM));
    }

    public void addPurpleSample() {
        float[] hsv = new float[3];
        Color.RGBToHSV(clamp(servoColor.red()), clamp(servoColor.green()), clamp(servoColor.blue()), hsv);
        purpleHues.add(hsv[0]);
        purpleDists.add((float)servoDistance.getDistance(DistanceUnit.CM));
    }

    public void reset() {
        greenHues.clear(); greenDists.clear();
        purpleHues.clear(); purpleDists.clear();
    }

    public int greenCount() { return greenHues.size(); }
    public int purpleCount() { return purpleHues.size(); }

    public boolean hasEnoughSamples() {
        return greenHues.size() >= 3 && purpleHues.size() >= 3
                && greenDists.size() >= 3 && purpleDists.size() >= 3;
    }

    public float greenHueMin() { return minList(greenHues); }
    public float greenHueMax() { return maxList(greenHues); }
    public float purpleHueMin() { return minList(purpleHues); }
    public float purpleHueMax() { return maxList(purpleHues); }

    public float greenDistMinCm() { return minList(greenDists); }
    public float greenDistMaxCm() { return maxList(greenDists); }
    public float purpleDistMinCm() { return minList(purpleDists); }
    public float purpleDistMaxCm() { return maxList(purpleDists); }

    public float computedHueMidpoint() {
        if (!hasEnoughSamples()) return Float.NaN;
        return circularMidpoint(meanHue(greenHues), meanHue(purpleHues));
    }

    public float computedDistanceThresholdCm() {
        if (!hasEnoughSamples()) return Float.NaN;
        return (meanFloat(greenDists) + meanFloat(purpleDists)) * 0.5f;
    }

    private float minList(List<Float> a) {
        if (a.isEmpty()) return Float.NaN;
        float m = Float.POSITIVE_INFINITY;
        for (float v : a) if (v < m) m = v;
        return m;
    }

    private float maxList(List<Float> a) {
        if (a.isEmpty()) return Float.NaN;
        float m = Float.NEGATIVE_INFINITY;
        for (float v : a) if (v > m) m = v;
        return m;
    }

    private float meanFloat(List<Float> a) {
        if (a.isEmpty()) return Float.NaN;
        double s = 0;
        for (float v : a) s += v;
        return (float)(s / a.size());
    }

    private float meanHue(List<Float> a) {
        if (a.isEmpty()) return Float.NaN;
        double sx = 0, sy = 0;
        for (float h : a) {
            double rad = Math.toRadians(h);
            sx += Math.cos(rad);
            sy += Math.sin(rad);
        }
        double ang = Math.atan2(sy, sx);
        float deg = (float)Math.toDegrees(ang);
        if (deg < 0) deg += 360.0f;
        return deg;
    }

    private float circularMidpoint(float a, float b) {
        float diff = ((b - a + 540.0f) % 360.0f) - 180.0f;
        float mid = (a + diff * 0.5f) % 360.0f;
        if (mid < 0) mid += 360.0f;
        return mid;
    }

    private int clamp(int v) {
        if (v < 0) return 0;
        if (v > 255) return 255;
        return v;
    }
}
