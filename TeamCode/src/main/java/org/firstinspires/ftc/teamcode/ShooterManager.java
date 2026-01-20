package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ReadWriteFile;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import java.io.File;

@Config
public class ShooterManager {
    // Tunable via FTC Dashboard
    public static double kP = 0.006, kI = 0.0, kD = 0.0001, kV = 0.00048, kS = 0.05;
    public static double PWM_MIN = 500, PWM_MAX = 2500;

    public final InterpLUT rpmTable = new InterpLUT();
    public final InterpLUT hoodTable = new InterpLUT();
    private final PIDFController controller = new PIDFController(kP, kI, kD, 0);
    private final File dataFile = AppUtil.getInstance().getSettingsFile("shooter_data.csv");

    public void tunePIDF(double p, double i, double d) { controller.setPIDF(p, i, d, 0); }

    public void saveShot(double dist, double rpm, double angle, boolean hit) {
        String current = dataFile.exists()? ReadWriteFile.readFile(dataFile) : "Dist,RPM,Angle,Hit\n";
        String line = String.format("%.2f,%.0f,%.1f,%b\n", dist, rpm, angle, hit);
        ReadWriteFile.writeFile(dataFile, current + line);
    }

    public void loadTrainingData() {
        if (!dataFile.exists()) return;
        String lines = ReadWriteFile.readFile(dataFile).split("\n");
        for (String line : lines) {
            String p = line.split(",");
            if (p.length < 4 ||!Boolean.parseBoolean(p[1])) continue; // Only load "Hits"
            rpmTable.add(Double.parseDouble(p), Double.parseDouble(p[2]));
            hoodTable.add(Double.parseDouble(p), Double.parseDouble(p[3]));
        }
        rpmTable.createLUT(); hoodTable.createLUT();
    }

    public void updateServo(ServoImplEx servo, double targetAngle) {
        servo.setPwmRange(new PwmControl.PwmRange(PWM_MIN, PWM_MAX));
        double pos = (targetAngle + 5.0) / 45.0; // Maps -5..40 range
        servo.setPosition(Math.max(0, Math.min(1, pos)));
    }

    public double getFlywheelPower(double dist, double currentRPM) {
        double target = rpmTable.get(dist);
        return controller.calculate(currentRPM, target) + (target * kV) + (Math.signum(target) * kS);
    }
}