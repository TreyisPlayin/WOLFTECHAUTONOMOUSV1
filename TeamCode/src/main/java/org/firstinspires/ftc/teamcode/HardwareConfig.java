package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

/**
 * Maps all hardware. Make sure these strings match your RC configuration.
 * FTC Javadoc for hardware interfaces: https://javadoc.io/doc/org.firstinspires.ftc  (RobotCore)
 */
public class HardwareConfig {
    // Drivetrain
    public DcMotor leftFront, leftBack, rightFront, rightBack;

    // Deadwheel encoders
    public DcMotor encoderLeft, encoderRight, encoderBack;

    // Intake & arm
    public DcMotor intakeMotor;
    public Servo   intakeArm;

    // Sensors
    public ColorSensor colorSensor;        // REV Color/Distance → ColorSensor API. :contentReference[oaicite:1]{index=1}
    public DistanceSensor distFront, distLeft, distRight, distBack; // DistanceSensor API. :contentReference[oaicite:2]{index=2}
    public TouchSensor touchLeft, touchRight;

    // Vision
    public WebcamName webcam;              // VisionPortal webcam device. :contentReference[oaicite:3]{index=3}

    public HardwareConfig(HardwareMap hw) {

        leftFront   = hw.get(DcMotor.class, "lf");
        leftBack    = hw.get(DcMotor.class, "lb");
        rightFront  = hw.get(DcMotor.class, "rf");
        rightBack   = hw.get(DcMotor.class, "rb");

        encoderLeft  = hw.get(DcMotor.class, "enc_left");
        encoderRight = hw.get(DcMotor.class, "enc_right");
        encoderBack  = hw.get(DcMotor.class, "enc_back");

        intakeMotor = hw.get(DcMotor.class, "intake");
        intakeArm   = hw.get(Servo.class,    "intake_arm");

        colorSensor = hw.get(ColorSensor.class, "color");
        distFront   = hw.get(DistanceSensor.class, "dist_front");
        distLeft    = hw.get(DistanceSensor.class, "dist_left");
        distRight   = hw.get(DistanceSensor.class, "dist_right");
        distBack    = hw.get(DistanceSensor.class, "dist_back");

        touchLeft   = hw.get(TouchSensor.class, "touch_left");
        touchRight  = hw.get(TouchSensor.class, "touch_right");

        webcam      = hw.get(WebcamName.class, "webcam");

        // Motor directions & braking as needed
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack .setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor m : new DcMotor[]{leftFront,leftBack,rightFront,rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
