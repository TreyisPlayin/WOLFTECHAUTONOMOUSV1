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
    public Servo LRPusher;
    public Servo compliantWheel;
    public Servo leftAuger;
    public Servo leftPusher;
    public Servo rightAuger;
    public Servo rightPusher;

    public HardwareConfig(LinearOpMode opmode) {currentOpMode = opmode;}

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        HardwareMap map = (hardwareMap != null) ? hardwareMap : currentOpMode.hardwareMap;
        if (map == null) {
            throw new IllegalStateException("HardwareMap unavailable during init");
        }

        // Pinpoint must be fetched before it can be used. Doing so here prevents a null pointer
        // if init() is called before the member is assigned (which would otherwise crash the OpMode).
        try {
            pinpoint = map.get(GoBildaPinpointDriver.class, "pinpointComputer");
            if (pinpoint != null) {
                pinpoint.resetPosAndIMU();
            }
        } catch (IllegalArgumentException e) {
            pinpoint = null;
            if (telemetry != null) {
                telemetry.addLine("Pinpoint not configured");
                telemetry.update();
            }
        }

        //huskyLens = currentOpMode.hardwareMap.get(HuskyLens.class, "huskyLens");
        leftFront = map.get(DcMotor.class, "LF");
        rightFront = map.get(DcMotor.class, "RF");
        leftRear = map.get(DcMotor.class, "LR");
        rightRear = map.get(DcMotor.class, "RR");
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
