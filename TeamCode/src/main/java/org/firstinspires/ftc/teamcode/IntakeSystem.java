package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * Controls intake rollers and detects objects by color.
 */
public class IntakeSystem {
    private final HardwareConfig robot;
    private final ColorSensor sensor;

    public IntakeSystem(HardwareConfig robot) {
        this.robot = robot;
        this.sensor = robot.colorSensor;
    }

    public void activateIntake() {
        robot.intakeMotor.setPower(1.0);
    }

    public void release() {
        robot.intakeMotor.setPower(-1.0);
        try { Thread.sleep(500);} catch (InterruptedException ignored){}
        robot.intakeMotor.setPower(0);
    }

    public boolean detectTargetColor() {
        int r=sensor.red(), g=sensor.green(), b=sensor.blue();
        return r>100 && g<80 && b<80;
    }
}
