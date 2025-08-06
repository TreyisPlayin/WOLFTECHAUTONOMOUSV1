package org.firstinspires.ftc.teamcode;

/**
 * Simple PID controller.
 */
public class PIDController {
    private final double kP, kI, kD;
    private double integral = 0, lastError = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Compute control output for given error.
     * Call approx. every loop cycle.
     */
    public double calculate(double error) {
        integral += error;
        double derivative = error - lastError;
        lastError = error;
        return kP * error + kI * integral + kD * derivative;
    }

    /** Reset internal state (e.g. between successive rotateTo or driveTo calls). */
    public void reset() {
        integral = 0;
        lastError = 0;
    }
}
