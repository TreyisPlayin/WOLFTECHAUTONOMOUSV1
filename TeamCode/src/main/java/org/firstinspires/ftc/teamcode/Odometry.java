package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Tracks robot pose via three dead-wheel encoders.
 */
public class Odometry {
    private final DcMotor leftEnc, rightEnc, backEnc;
    private static final double TPR=8192, WD=2.0, CIRC=Math.PI*WD, TW=12.0;
    private int l0=0, r0=0, b0=0;
    private double x=0, y=0, heading=0;
    private final ElapsedTime timer=new ElapsedTime();

    public Odometry(HardwareConfig hw) {
        this.leftEnc  = hw.encoderLeft;
        this.rightEnc = hw.encoderRight;
        this.backEnc  = hw.encoderBack;
        init();
    }
    private void init() {
        leftEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEnc.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backEnc.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void update() {
        int l=leftEnc.getCurrentPosition(),
                r=rightEnc.getCurrentPosition(),
                b=backEnc.getCurrentPosition();
        int dl=l-l0, dr=r-r0, db=b-b0;
        l0=l; r0=r; b0=b;
        double inL=dl/TPR*CIRC, inR=dr/TPR*CIRC, inB=db/TPR*CIRC;
        double dTheta=(inR-inL)/TW; heading=(heading+dTheta+Math.PI)%(2*Math.PI)-Math.PI;
        double fwd=(inL+inR)/2, lat=inB;
        double c=Math.cos(heading), s=Math.sin(heading);
        x+=fwd*c - lat*s; y+=fwd*s + lat*c;
    }
    public Position getCurrentPosition() {
        return new Position(x, y, Math.toDegrees(heading));
    }
}
