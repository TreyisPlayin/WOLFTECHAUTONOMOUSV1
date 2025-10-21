package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;




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
    public Servo LRPusher;
    public Servo compliantWheel;
    public Servo leftAuger;
    public Servo leftPusher;
    public Servo rightAuger;
    public Servo rightPusher;

    public HardwareConfig(LinearOpMode opmode) {currentOpMode = opmode;}

    public void init() {
        pinpoint.resetPosAndIMU();
        //huskyLens = currentOpMode.hardwareMap.get(HuskyLens.class, "huskyLens");
        pinpoint = currentOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpointComputer");
        leftFront = currentOpMode.hardwareMap.get(DcMotor.class, "LF");
        rightFront = currentOpMode.hardwareMap.get(DcMotor.class, "RF");
        leftRear = currentOpMode.hardwareMap.get(DcMotor.class, "LR");
        rightRear = currentOpMode.hardwareMap.get(DcMotor.class, "RR");
        //leftCatapult = currentOpMode.hardwareMap.get(DcMotor.class, "CataL");
        //rightCatapult = currentOpMode.hardwareMap.get(DcMotor.class, "CataR");
        //LRPusher = currentOpMode.hardwareMap.get(Servo.class, "pushLR");
        //compliantWheel = currentOpMode.hardwareMap.get(Servo.class, "compliantWheel");
        //leftAuger = currentOpMode.hardwareMap.get(Servo.class, "leftAuger");
        //leftPusher = currentOpMode.hardwareMap.get(Servo.class, "leftPusher");
        //rightAuger = currentOpMode.hardwareMap.get(Servo.class, "rightAuger");
        //rightPusher = currentOpMode.hardwareMap.get(Servo.class, "rightPusher");

    }

}
