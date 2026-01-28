package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.dfrobot.HuskyLens;

public class RobotHardware {
    // LED PWM Variables for REV Blinkin
    public static float pwmRed = 0.279f;   // Solid Red pulse width
    public static float pwmGreen = 0.388f; // Solid Green pulse width
    public static float pwmOff = 0.0f;

    // Standard Hardware
    public DcMotorEx flywheel;
    public ServoImplEx hood, blinkin;
    public TouchSensor launchTrigger;
    public Limelight3A limelight;

    // Intake Hardware (from DECODEPredictiveIntake)
    public CRServo intakeServo, leftAuger, rightAuger;
    public Servo lrPusher, lPush, rPush;
    public DigitalChannel siloL, siloR;

    public void init(HardwareMap hwMap) {
        // Core Shooter Hardware
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        hood = (ServoImplEx) hwMap.get(Servo.class, "hood");
        blinkin = (ServoImplEx) hwMap.get(Servo.class, "Blinkin");
        blinkin.setPwmRange(new PwmControl.PwmRange(500, 2500)); // Standard Blinkin Range
        launchTrigger = hwMap.get(TouchSensor.class, "launchTrigger");
        limelight = hwMap.get(Limelight3A.class, "limelight");

        // Predictive Intake Hardware
        intakeServo = hwMap.get(CRServo.class, "intake servo");
        leftAuger = hwMap.get(CRServo.class, "leftAuger");
        rightAuger = hwMap.get(CRServo.class, "rightAuger");
        lrPusher = hwMap.get(Servo.class, "LR pusher");
        lPush = hwMap.get(Servo.class, "Lpusher");
        rPush = hwMap.get(Servo.class, "Rpusher");
        siloL = hwMap.get(DigitalChannel.class, "distL");
        siloR = hwMap.get(DigitalChannel.class, "distR");
    }

    public void setLed(float pwmValue) { blinkin.setPosition(pwmValue); }
}