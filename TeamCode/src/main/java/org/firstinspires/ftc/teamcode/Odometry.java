package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Dead-wheel odometry (three encoders) for pose estimation.
 * UPDATED: added setPose(...) so AprilTag can correct drift.
 */
public class Odometry {
    private final DcMotor leftEnc, rightEnc, backEnc;
    // --- TUNE THESE ---
    private static final double TPR = 8192;      // ticks per rev of your deadwheel encoders
    private static final double WD  = 2.0;       // deadwheel diameter in inches
    private static final double CIRC = Math.PI * WD;
    private static final double TW  = 12.0;      // track width (distance between L/R deadwheels), in inches

    private int l0=0, r0=0, b0=0;
    private double x=0, y=0, headingRad=0.0;
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
        int cl = leftEnc.getCurrentPosition();
        int cr = rightEnc.getCurrentPosition();
        int cb = backEnc.getCurrentPosition();
        int dl = cl - l0, dr = cr - r0, db = cb - b0;
        l0 = cl; r0 = cr; b0 = cb;

        double inL = dl/TPR*CIRC, inR = dr/TPR*CIRC, inB = db/TPR*CIRC;
        double dTheta = (inR - inL) / TW;               // radians
        headingRad = wrapRad(headingRad + dTheta);

        double fwd = (inL + inR) / 2.0;
        double lat = inB;
        double c = Math.cos(headingRad), s = Math.sin(headingRad);
        x += fwd*c - lat*s;
        y += fwd*s + lat*c;
    }

    /** Allow external correction (e.g., AprilTag). */
    public void setPose(Position p) {
        this.x = p.getX();
        this.y = p.getY();
        this.headingRad = Math.toRadians(p.getHeading());
    }

    public Position getCurrentPosition() {
        return new Position(x, y, Math.toDegrees(headingRad));
    }

    private static double wrapRad(double a){
        // wrap to (-π, π]
        a = (a + Math.PI) % (2*Math.PI);
        if (a <= 0) a += 2*Math.PI;
        return a - Math.PI;
    }
}
