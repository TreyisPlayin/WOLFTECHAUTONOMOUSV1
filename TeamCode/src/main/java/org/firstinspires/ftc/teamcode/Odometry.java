package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Dead-wheel odometry (three encoders) for pose estimation.
 */
public class Odometry {
    private final DcMotor leftEnc, rightEnc, backEnc;
    private static final double TPR = 8192, WD = 2.0, CIRC = Math.PI*WD, TW = 12.0;
    private int l0=0, r0=0, b0=0;
    private double x=0, y=0, heading=0;
    private final ElapsedTime timer = new ElapsedTime();

    public Odometry(HardwareConfig hw) {
        leftEnc  = hw.encoderLeft;
        rightEnc = hw.encoderRight;
        backEnc  = hw.encoderBack;
        init();
    }

    private void init() {
        leftEnc .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEnc .setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEnc .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backEnc .setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /** Call in your loop to update (x,y,heading). */
    public void update() {
        int cl = leftEnc.getCurrentPosition(),
                cr = rightEnc.getCurrentPosition(),
                cb = backEnc.getCurrentPosition();
        int dl = cl - l0, dr = cr - r0, db = cb - b0;
        l0 = cl; r0 = cr; b0 = cb;

        double inL = dl/TPR*CIRC, inR = dr/TPR*CIRC, inB = db/TPR*CIRC;
        double dTheta = (inR-inL)/TW;
        heading = ((heading + dTheta + Math.PI) % (2*Math.PI)) - Math.PI;

        double fwd = (inL + inR)/2, lat = inB;
        double cos = Math.cos(heading), sin = Math.sin(heading);
        x += fwd*cos - lat*sin;
        y += fwd*sin + lat*cos;
    }

    public Position getCurrentPosition() {
        return new Position(x, y, Math.toDegrees(heading));
    }
}
