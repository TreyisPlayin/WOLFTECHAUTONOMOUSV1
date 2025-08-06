package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * HardwareConfig centralizes all hardware initialization.
 * After construction, each subsystem can reference robot.leftFront, etc.
 */
public class HardwareConfig {
    // Drive motors
    public DcMotor leftFront, leftBack, rightFront, rightBack;

    // Intake
    public DcMotor intakeMotor;
    public Servo  intakeArm;

    // Odometry (dead wheels)
    public DcMotor encoderLeft, encoderRight, encoderBack;

    // Sensors
    public ColorSensor colorSensor;
    public DistanceSensor frontDistance, leftDistance, rightDistance, backDistance;

    /** Call in your OpMode’s init to map names to hardware. */
    public HardwareConfig(HardwareMap hwMap) {
        // Drive motors
        leftFront  = hwMap.get(DcMotor.class, "leftFront");
        leftBack   = hwMap.get(DcMotor.class, "leftBack");
        rightFront = hwMap.get(DcMotor.class, "rightFront");
        rightBack  = hwMap.get(DcMotor.class, "rightBack");

        // Intake
        intakeMotor = hwMap.get(DcMotor.class, "intakeMotor");
        intakeArm   = hwMap.get(Servo.class,  "intakeArm");

        // Odometry encoders
        encoderLeft  = hwMap.get(DcMotor.class, "encoderLeft");
        encoderRight = hwMap.get(DcMotor.class, "encoderRight");
        encoderBack  = hwMap.get(DcMotor.class, "encoderBack");

        // Sensors
        colorSensor    = hwMap.get(ColorSensor.class,    "colorSensor");
        frontDistance  = hwMap.get(DistanceSensor.class, "frontDistance");
        leftDistance   = hwMap.get(DistanceSensor.class, "leftDistance");
        rightDistance  = hwMap.get(DistanceSensor.class, "rightDistance");
        backDistance   = hwMap.get(DistanceSensor.class, "backDistance");

        // Motor direction adjustments (example)
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack .setDirection(DcMotor.Direction.REVERSE);

        // Optionally, set zero-power behavior
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
