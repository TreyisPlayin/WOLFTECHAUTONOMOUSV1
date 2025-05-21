package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * IntakeSystem.java
 *
 * Automatic grab helper: call autoGrab() when aligned.
 */
public class IntakeSystem {
    private final HardwareConfig robot;
    private final ColorSensor color;

    public IntakeSystem(HardwareConfig hw) {
        this.robot = hw;
        this.color = hw.frontSensor; // or dedicated color sensor
    }

    /** Auto-grab: spin intake until object detected up close. */
    public void autoGrab() {
        robot.intakeMotor.setPower(1.0);
        // you could wait for a beam-break or timeout
        sleep(500);
        robot.intakeMotor.setPower(0);
    }

    /** Release object */
    public void release() {
        robot.intakeMotor.setPower(-1.0);
        sleep(500);
        robot.intakeMotor.setPower(0);
    }

    private void sleep(long ms) { try { Thread.sleep(ms); } catch (InterruptedException ignored){} }
}
