package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ShooterManager {

    private final HardwareConfig hw;
    private final Telemetry telemetry;

    private double targetRPM = 0.0;

    private double gateClosed = 0.0;
    private double gateOpen   = 1.0;
    private double pusherHome = 0.0;
    private double pusherFire = 1.0;

    private static final long PUSHER_TIME_MS = 200;

    // Always-on feed power per your request (adjust)
    private static final double FEED_IDLE_POWER = 0.35;

    // Exit detect
    private static final double EXIT_BLOCKED_MM = 80.0;
    private boolean exitWasBlocked = false;
    private boolean shotJustExited = false;

    public ShooterManager(HardwareConfig hw, Telemetry tel) {
        this.hw = hw;
        this.telemetry = tel;

        if (hw.flywheelMotor != null) {
            hw.flywheelMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            hw.flywheelMotor.setPower(0.0);
        }
        if (hw.forwardFeedMotor != null) {
            // always spinning
            hw.forwardFeedMotor.setPower(FEED_IDLE_POWER);
        }

        if (hw.leftGate != null) hw.leftGate.setPosition(gateClosed);
        if (hw.rightGate != null) hw.rightGate.setPosition(gateClosed);
        if (hw.leftPusher != null) hw.leftPusher.setPosition(pusherHome);
        if (hw.rightPusher != null) hw.rightPusher.setPosition(pusherHome);

        if (hw.blinkin != null) hw.blinkin.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
        if (hw.flywheelMotor != null) hw.flywheelMotor.setVelocity(rpmToTicksPerSecond(rpm));
        // Replace this mapping with your own equation if desired
        if (hw.hoodServo != null) hw.hoodServo.setPosition(getAngleForRPM(rpm));
    }

    private double getAngleForRPM(double rpm) {
        double minRPM = 2000.0, maxRPM = 4000.0;
        double minPos = 0.4,   maxPos = 0.9;
        if (rpm < minRPM) rpm = minRPM;
        if (rpm > maxRPM) rpm = maxRPM;
        double t = (rpm - minRPM) / (maxRPM - minRPM);
        return minPos + t * (maxPos - minPos);
    }

    public void waitUntilReady() {
        if (hw.flywheelMotor == null) return;
        double targetTPS = rpmToTicksPerSecond(targetRPM);
        ElapsedTime timer = new ElapsedTime();
        while (timer.milliseconds() < 2000) {
            double v = hw.flywheelMotor.getVelocity();
            telemetry.addData("Shooter vel", "%.0f / %.0f", v, targetTPS);
            telemetry.update();
            if (Math.abs(v - targetTPS) <= 0.05 * targetTPS) break;
        }
    }

    public void setLedReady(boolean ready) {
        if (hw.blinkin == null) return;
        hw.blinkin.setPattern(
                ready ? RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_GREEN
                        : RevBlinkinLedDriver.BlinkinPattern.BREATH_RED_ORANGE
        );
    }

    /** Must be called each loop to update exit detection & LED state if you want live feedback. */
    public void update() {
        shotJustExited = false;
        if (hw.flywheelExitSensor != null) {
            double d = hw.flywheelExitSensor.getDistance(DistanceUnit.MM);
            boolean blocked = (d > 0 && d < EXIT_BLOCKED_MM);
            if (exitWasBlocked && !blocked) shotJustExited = true; // edge → ball left
            exitWasBlocked = blocked;
        }
    }

    public boolean shotJustExited() {
        return shotJustExited;
    }

    /** Fire one ball from given channel (0=LEFT, 1=RIGHT). Gates + per-channel pusher. */
    public void fireOneFromChannel(int channel) {
        Servo gate = (channel == 0) ? hw.leftGate  : hw.rightGate;
        Servo push = (channel == 0) ? hw.leftPusher: hw.rightPusher;
        if (gate == null || push == null) return;

        // open gate and pulse pusher to drop into the (already spinning) feed
        gate.setPosition(gateOpen);
        push.setPosition(pusherFire);
        sleep(PUSHER_TIME_MS);
        push.setPosition(pusherHome);
        gate.setPosition(gateClosed);
    }

    public void stopAll() {
        if (hw.flywheelMotor != null) hw.flywheelMotor.setPower(0.0);
        if (hw.forwardFeedMotor != null) hw.forwardFeedMotor.setPower(0.0);
        if (hw.leftGate != null) hw.leftGate.setPosition(gateClosed);
        if (hw.rightGate != null) hw.rightGate.setPosition(gateClosed);
        if (hw.leftPusher != null) hw.leftPusher.setPosition(pusherHome);
        if (hw.rightPusher != null) hw.rightPusher.setPosition(pusherHome);
        setLedReady(false);
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * 28.0 / 60.0;
    }

    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (InterruptedException ignored) {}
    }
}
