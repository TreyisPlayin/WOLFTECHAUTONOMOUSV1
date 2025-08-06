package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Odometry tracks the robot’s pose (x,y,heading) using three deadwheel encoders.
 * It converts encoder ticks to inches and integrates motion over time.
 */
public class Odometry {
    // Hardware encoders
    private final DcMotor leftEncoder, rightEncoder, backEncoder;

    // Encoder of ticks per wheel revolution and wheel circumference
    private static final double TICKS_PER_REV = 8192;
    private static final double WHEEL_DIAMETER_INCHES = 2.0;
    private static final double CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_INCHES;

    // Distance between left and right encoders (track width)
    private static final double TRACK_WIDTH = 12.0; // inches

    // Previous tick readings
    private int lastLeft = 0, lastRight = 0, lastBack = 0;

    // Robot pose
    private double x = 0, y = 0;
    private double heading = 0; // radians

    // Timer (optional) to throttle updates
    private final ElapsedTime timer = new ElapsedTime();

    public Odometry(HardwareConfig hw) {
        this.leftEncoder  = hw.encoderLeft;
        this.rightEncoder = hw.encoderRight;
        this.backEncoder  = hw.encoderBack;

        // Reset and run without encoder if needed
        initEncoders();
    }

    private void initEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastLeft = lastRight = lastBack = 0;
    }

    /**
     * Call this in your loop to update pose.
     */
    public void update() {
        int curL = leftEncoder.getCurrentPosition();
        int curR = rightEncoder.getCurrentPosition();
        int curB = backEncoder.getCurrentPosition();

        int dL = curL - lastLeft;
        int dR = curR - lastRight;
        int dB = curB - lastBack;

        lastLeft  = curL;
        lastRight = curR;
        lastBack  = curB;

        // Convert ticks to inches
        double inL = dL / TICKS_PER_REV * CIRCUMFERENCE;
        double inR = dR / TICKS_PER_REV * CIRCUMFERENCE;
        double inB = dB / TICKS_PER_REV * CIRCUMFERENCE;

        // Change in heading
        double dTheta = (inR - inL) / TRACK_WIDTH;
        heading += dTheta;
        // Normalize heading between -π to π
        heading = ((heading + Math.PI) % (2*Math.PI)) - Math.PI;

        // Forward movement is average of left/right
        double forward = (inL + inR) / 2.0;
        // Lateral movement from back wheel
        double lateral = inB;

        // Rotate local movements into global frame
        double cosH = Math.cos(heading);
        double sinH = Math.sin(heading);
        x += forward * cosH - lateral * sinH;
        y += forward * sinH + lateral * cosH;
    }

    /** @return Current estimated Position in inches & degrees. */
    public Position getCurrentPosition() {
        return new Position(x, y, Math.toDegrees(heading));
    }
}
