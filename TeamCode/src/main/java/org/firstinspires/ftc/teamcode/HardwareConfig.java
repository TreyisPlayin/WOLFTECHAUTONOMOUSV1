package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


public class HardwareConfig {

    private LinearOpMode currentOpMode = null;

    // ==== Localization / Vision ====
    public GoBildaPinpointDriver pinpoint;
    public HuskyLens huskyLens;

    // ==== Drivetrain ====
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;

    // ==== Intake / Funnels ====
    // CR servo roller at the bottom intake
    public CRServo intakeRoller;

    // Color sensors (total = 4)
    //  - two at the bottom of each funnel
    //  - one looking at the center pusher area
    //  - one looking toward the back of the channel
    public ColorSensor bottomColorLeft;
    public ColorSensor bottomColorRight;
    public ColorSensor centerPusherColor;   // at the central pusher region
    public ColorSensor rearChannelColor;    // toward the back of the channel

    // Top-of-intake goBILDA analog sensors (voltage) — one per channel
    public AnalogInput topAnalogLeft;
    public AnalogInput topAnalogRight;

    // Channel augers (CR servos)
    public CRServo leftAuger;
    public CRServo rightAuger;

    // Channel gates (standard servos)
    public Servo leftGate;
    public Servo rightGate;

    // Channel pushers (standard servos) — push into top feed/flywheel path
    public Servo leftPusher;
    public Servo rightPusher;

    // Central pusher (standard servo) that diverts into LEFT/RIGHT channels
    public Servo centerPusher;

    // Optional additional intake gate at the entrance
    public Servo intakeGate;

    // ==== Shooter ====
    public DcMotorEx flywheelMotor;        // flywheel
    public DcMotorEx forwardFeedMotor;     // top pusher/feed motor (always spinning low)
    public Servo hoodServo;                // angle adjuster

    // REV distance at flywheel exit to detect launch
    public DistanceSensor flywheelExitSensor;

    // LEDs
    public RevBlinkinLedDriver blinkin;

    public HardwareConfig(LinearOpMode opmode) {
        currentOpMode = opmode;
    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        // Localization
        pinpoint = currentOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpointComputer");
        pinpoint.resetPosAndIMU();
        huskyLens = currentOpMode.hardwareMap.get(HuskyLens.class, "huskyLens");

        // Drivetrain
        leftFront  = currentOpMode.hardwareMap.get(DcMotor.class, "LF");
        rightFront = currentOpMode.hardwareMap.get(DcMotor.class, "RF");
        leftRear   = currentOpMode.hardwareMap.get(DcMotor.class, "LR");
        rightRear  = currentOpMode.hardwareMap.get(DcMotor.class, "RR");

        // Common to reverse right side (adjust if your wiring differs)
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake / funnels
        intakeRoller       = currentOpMode.hardwareMap.get(CRServo.class, "intakeRoller");

        bottomColorLeft    = currentOpMode.hardwareMap.get(ColorSensor.class, "bottomLeft");
        bottomColorRight   = currentOpMode.hardwareMap.get(ColorSensor.class, "bottomRight");
        centerPusherColor  = currentOpMode.hardwareMap.get(ColorSensor.class, "centerColor");
        rearChannelColor   = currentOpMode.hardwareMap.get(ColorSensor.class, "rearColor");

        topAnalogLeft      = currentOpMode.hardwareMap.get(AnalogInput.class, "topAnalogL");
        topAnalogRight     = currentOpMode.hardwareMap.get(AnalogInput.class, "topAnalogR");

        leftAuger          = currentOpMode.hardwareMap.get(CRServo.class,   "leftAuger");
        rightAuger         = currentOpMode.hardwareMap.get(CRServo.class,   "rightAuger");

        leftGate           = currentOpMode.hardwareMap.get(Servo.class,     "gateL");
        rightGate          = currentOpMode.hardwareMap.get(Servo.class,     "gateR");

        leftPusher         = currentOpMode.hardwareMap.get(Servo.class,     "leftPusher");
        rightPusher        = currentOpMode.hardwareMap.get(Servo.class,     "rightPusher");

        centerPusher       = currentOpMode.hardwareMap.get(Servo.class,     "centerPusher");
        intakeGate         = currentOpMode.hardwareMap.get(Servo.class,     "intakeGate");

        // Shooter
        flywheelMotor      = (DcMotorEx) currentOpMode.hardwareMap.get(DcMotor.class, "flywheel");
        forwardFeedMotor   = (DcMotorEx) currentOpMode.hardwareMap.get(DcMotor.class, "feedTop");
        hoodServo          = currentOpMode.hardwareMap.get(Servo.class,       "hood");

        flywheelExitSensor = currentOpMode.hardwareMap.get(DistanceSensor.class, "exitToF");

        // LEDs
        blinkin            = currentOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");

        telemetry.addLine("HardwareConfig initialized (CR roller/augers, 4x color, 2x analog top, exit ToF)");
        telemetry.update();
    }
}
