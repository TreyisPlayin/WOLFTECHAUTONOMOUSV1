package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

/**
 * Map hardware names to objects; adjust names to your RC config.
 */
public class HardwareConfig {
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public DcMotor encoderLeft, encoderRight, encoderBack;
    public DcMotor intakeMotor;
    public ColorSensor colorSensor;
    public DistanceSensor frontDistance, leftDistance, rightDistance, backDistance;
    public Servo intakeArm;

    public HardwareConfig(HardwareMap hw) {
        leftFront   = hw.get(DcMotor.class, "left_front");
        leftBack    = hw.get(DcMotor.class, "left_back");
        rightFront  = hw.get(DcMotor.class, "right_front");
        rightBack   = hw.get(DcMotor.class, "right_back");

        encoderLeft  = hw.get(DcMotor.class, "enc_left");
        encoderRight = hw.get(DcMotor.class, "enc_right");
        encoderBack  = hw.get(DcMotor.class, "enc_back");

        intakeMotor = hw.get(DcMotor.class, "intake");
        intakeArm   = hw.get(Servo.class,    "intake_arm");

        colorSensor   = hw.get(ColorSensor.class,    "color");
        frontDistance = hw.get(DistanceSensor.class, "dist_front");
        leftDistance  = hw.get(DistanceSensor.class, "dist_left");
        rightDistance = hw.get(DistanceSensor.class, "dist_right");
        backDistance  = hw.get(DistanceSensor.class, "dist_back");

        // Reverse right side if needed:
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack .setDirection(DcMotor.Direction.REVERSE);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
