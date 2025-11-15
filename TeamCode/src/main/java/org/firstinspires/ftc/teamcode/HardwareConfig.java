package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class HardwareConfig {
    private LinearOpMode currentOpMode = null;
    public HuskyLens huskyLens;
    public GoBildaPinpointDriver pinpoint;
    public DcMotor leftFront;
    public DcMotor leftRear;
    public DcMotor rightFront;
    public DcMotor rightRear;
    public DcMotor leftCatapult;
    public DcMotor rightCatapult;
    public DcMotor intakeMotor;
    public Servo LRPusher;
    public Servo compliantWheel;
    public Servo leftAuger;
    public Servo leftPusher;
    public Servo rightAuger;
    public Servo rightPusher;

    public HardwareConfig(LinearOpMode opmode) {currentOpMode = opmode;}

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        // Initialize sensors first so they can be used immediately by the calling OpMode.
        //huskyLens = currentOpMode.hardwareMap.get(HuskyLens.class, "huskyLens");
        pinpoint = currentOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpointComputer");
        if (pinpoint != null) {
            pinpoint.resetPosAndIMU();
        }
        leftFront = currentOpMode.hardwareMap.get(DcMotor.class, "LF");
        rightFront = currentOpMode.hardwareMap.get(DcMotor.class, "RF");
        leftRear = currentOpMode.hardwareMap.get(DcMotor.class, "LR");
        rightRear = currentOpMode.hardwareMap.get(DcMotor.class, "RR");

        leftCatapult = currentOpMode.hardwareMap.tryGet(DcMotor.class, "CataL");
        rightCatapult = currentOpMode.hardwareMap.tryGet(DcMotor.class, "CataR");
        if (leftCatapult == null) {
            leftCatapult = currentOpMode.hardwareMap.tryGet(DcMotor.class, "leftCatapult");
        }
        if (rightCatapult == null) {
            rightCatapult = currentOpMode.hardwareMap.tryGet(DcMotor.class, "rightCatapult");
        }
        if (leftCatapult != null) {
            leftCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (rightCatapult != null) {
            rightCatapult.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        intakeMotor = currentOpMode.hardwareMap.tryGet(DcMotor.class, "intake");
        if (intakeMotor == null) {
            intakeMotor = currentOpMode.hardwareMap.tryGet(DcMotor.class, "intakeMotor");
        }
        if (intakeMotor != null) {
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        //LRPusher = currentOpMode.hardwareMap.get(Servo.class, "pushLR");
        //compliantWheel = currentOpMode.hardwareMap.get(Servo.class, "compliantWheel");
        //leftAuger = currentOpMode.hardwareMap.get(Servo.class, "leftAuger");
        //leftPusher = currentOpMode.hardwareMap.get(Servo.class, "leftPusher");
        //rightAuger = currentOpMode.hardwareMap.get(Servo.class, "rightAuger");
        //rightPusher = currentOpMode.hardwareMap.get(Servo.class, "rightPusher");

    }

}
