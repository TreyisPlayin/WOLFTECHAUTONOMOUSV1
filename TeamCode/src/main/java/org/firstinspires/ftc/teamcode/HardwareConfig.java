package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;

/** Initializes all motors, servos, sensors by name. */
public class HardwareConfig {
    public DcMotor leftFront, leftBack, rightFront, rightBack;
    public DcMotor intakeMotor;
    public Servo  intakeArm;
    public DcMotor encoderLeft, encoderRight, encoderBack;
    public ColorSensor colorSensor;
    public DistanceSensor frontDistance, leftDistance, rightDistance, backDistance;

    public HardwareConfig(HardwareMap hw) {
        leftFront  = hw.get(DcMotor.class,"leftFront");
        leftBack   = hw.get(DcMotor.class,"leftBack");
        rightFront = hw.get(DcMotor.class,"rightFront");
        rightBack  = hw.get(DcMotor.class,"rightBack");
        intakeMotor= hw.get(DcMotor.class,"intakeMotor");
        intakeArm  = hw.get(Servo.class, "intakeArm");
        encoderLeft= hw.get(DcMotor.class,"encoderLeft");
        encoderRight=hw.get(DcMotor.class,"encoderRight");
        encoderBack= hw.get(DcMotor.class,"encoderBack");
        colorSensor= hw.get(ColorSensor.class,   "colorSensor");
        frontDistance=hw.get(DistanceSensor.class,"frontDistance");
        leftDistance =hw.get(DistanceSensor.class,"leftDistance");
        rightDistance=hw.get(DistanceSensor.class,"rightDistance");
        backDistance =hw.get(DistanceSensor.class,"backDistance");
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack .setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
