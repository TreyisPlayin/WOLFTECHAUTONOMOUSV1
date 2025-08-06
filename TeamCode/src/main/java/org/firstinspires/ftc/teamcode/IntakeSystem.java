package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * IntakeSystem controls rollers or arms to pick up and release objects.
 * It also uses a ColorSensor to detect if the target is present.
 */
public class IntakeSystem {

    private final HardwareConfig robot;
    private final ColorSensor colorSensor;

    public IntakeSystem(HardwareConfig robot) {
        this.robot = robot;
        this.colorSensor = robot.colorSensor;
    }

    /**
     * Runs intake rollers inward to grab objects.
     */
    public void activateIntake() {
        robot.intakeMotor.setPower(1.0);
    }

    /** Reverses rollers to eject objects. */
    public void release() {
        robot.intakeMotor.setPower(-1.0);
        try { Thread.sleep(500); } catch (InterruptedException ignored) {}
        robot.intakeMotor.setPower(0);
    }

    /**
     * Uses a simple red-color threshold to detect game element.
     * @return true if red-dominant object is seen.
     */
    public boolean detectTargetColor() {
        int red   = colorSensor.red();
        int green = colorSensor.green();
        int blue  = colorSensor.blue();
        return red > 100 && green < 80 && blue < 80;
    }
}
