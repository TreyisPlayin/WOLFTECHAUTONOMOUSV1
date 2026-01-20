package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.dfrobot.HuskyLens;

public class RobotHardware {
    // PWM signal values for LED colors (0.0 to 1.0 positions)
    public static final double LED_RED = 0.279;
    public static final double LED_GREEN = 0.388;
    public static final double LED_OFF = 0.0;

    public DcMotorEx flywheel, pusher, intakeL, intakeR;
    public ServoImplEx hood, leftGate, rightGate, ledStrip;
    public TouchSensor launchTrigger;
    public DistanceSensor distL, distR;
    public Limelight3A limelight;
    public HuskyLens huskyLens;

    public void init(HardwareMap hwMap) {
        // Motors
        flywheel = hwMap.get(DcMotorEx.class, "flywheel");
        pusher = hwMap.get(DcMotorEx.class, "pusher");
        intakeL = hwMap.get(DcMotorEx.class, "intakeL");
        intakeR = hwMap.get(DcMotorEx.class, "intakeR");

        // Servos (Cast to ServoImplEx for PWM Range control)
        hood = (ServoImplEx) hwMap.get(Servo.class, "hood");
        leftGate = (ServoImplEx) hwMap.get(Servo.class, "leftGate");
        rightGate = (ServoImplEx) hwMap.get(Servo.class, "rightGate");
        ledStrip = (ServoImplEx) hwMap.get(Servo.class, "ledStrip");
        ledStrip.setPwmRange(new PwmControl.PwmRange(500, 2500));

        // Sensors
        launchTrigger = hwMap.get(TouchSensor.class, "launchTrigger");
        distL = hwMap.get(DistanceSensor.class, "distL");
        distR = hwMap.get(DistanceSensor.class, "distR");
        limelight = hwMap.get(Limelight3A.class, "limelight");
        huskyLens = hwMap.get(HuskyLens.class, "huskylens");
    }

    public void setLed(double pwmSignal) { ledStrip.setPosition(pwmSignal); }
}