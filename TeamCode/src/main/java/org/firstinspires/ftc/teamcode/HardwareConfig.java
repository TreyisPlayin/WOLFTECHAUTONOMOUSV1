package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Maps all robot hardware ports to variables
 * and provides basic drive methods.
 */
public class HardwareConfig {
    public DcMotor leftDrive, rightDrive, intakeMotor;
    public DistanceSensor frontSensor, backSensor, leftSensor, rightSensor;

    public HardwareConfig(HardwareMap hw) {
        leftDrive   = hw.get(DcMotor.class, "leftDrive");
        rightDrive  = hw.get(DcMotor.class, "rightDrive");
        intakeMotor = hw.get(DcMotor.class, "intakeMotor");
        frontSensor = hw.get(DistanceSensor.class, "frontSensor");
        backSensor  = hw.get(DistanceSensor.class, "backSensor");
        leftSensor  = hw.get(DistanceSensor.class, "leftSensor");
        rightSensor = hw.get(DistanceSensor.class, "rightSensor");
    }

    /** Tank drive */
    public void drive(double leftPow, double rightPow) {
        leftDrive.setPower(leftPow);
        rightDrive.setPower(rightPow);
    }

    /** Stop all motion */
    public void stopAll() {
        drive(0,0);
        intakeMotor.setPower(0);
    }
}
