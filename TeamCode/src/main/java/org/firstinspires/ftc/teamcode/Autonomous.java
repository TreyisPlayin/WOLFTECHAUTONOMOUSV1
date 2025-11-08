package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "MainAuto_FieldNavExample", group = "Auto")
public class Autonomous extends LinearOpMode {

    private enum State {
        TO_START,
        MOVE1, MOVE2, MOVE3, MOVE4, MOVE5,
        DONE
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // 1) Hardware init
        HardwareConfig hw = new HardwareConfig(this);
        hw.init(hardwareMap, telemetry);  // <— adjust to your real signature

        // 2) FieldNav: Pinpoint-first + AprilTag correction
        FieldNav nav = new FieldNav(hardwareMap, telemetry, hw.pinpoint, "Webcam 1");
        nav.setAlliance(FieldNav.Alliance.BLUE); // or RED this match

        telemetry.addLine("FieldNav ready. Waiting for start...");
        telemetry.update();

        // 3) Waypoints (FTC field frame; inches)
        final double[] START = new double[]{ -36.0, -60.0 };
        final double[] P1    = new double[]{ -12.0, -36.0 };
        final double[] P2    = new double[]{  12.0, -12.0 };
        final double[] P3    = new double[]{  24.0,  12.0 };
        final double[] P4    = new double[]{  36.0,  36.0 };
        final double[] P5    = new double[]{  12.0,  48.0 };

        // Optional final headings (deg). Use null to skip end-facing for a leg.
        final Double H_START = 90.0;
        final Double H1 = null, H2 = 0.0, H3 = null, H4 = -90.0, H5 = 180.0;

        // 4) Prime first leg (to START)
        State state = State.TO_START;
        nav.driveTo(START[0], START[1], H_START);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // NAV update
            boolean arrived = nav.update();
            FieldNav.DriveCommand dc = nav.getDriveCommand();

            // Basic mecanum mix (robot-frame)
            double lf = dc.forward + dc.strafe + dc.rotate;
            double rf = dc.forward - dc.strafe - dc.rotate;
            double lr = dc.forward - dc.strafe + dc.rotate;
            double rr = dc.forward + dc.strafe - dc.rotate;
            double m = Math.max(1.0, Math.max(Math.max(Math.abs(lf), Math.abs(rf)),
                    Math.max(Math.abs(lr), Math.abs(rr))));
            hw.leftFront.setPower(lf / m);
            hw.rightFront.setPower(rf / m);
            hw.leftRear.setPower(lr / m);
            hw.rightRear.setPower(rr / m);

            // Telemetry (optional)
            FieldNav.TagDelta d = nav.getGoalTagDelta();
            FieldNav.Pose2d pose = nav.getPose();
            telemetry.addData("State", state);
            telemetry.addData("Pose", "(%.1f, %.1f)  %.1f°", pose.x, pose.y, pose.hDeg);
            telemetry.addData("GoalTag Δ (cam)", "x=%.1f in, y=%.1f in, seen=%s", d.xIn, d.yIn, d.valid);
            telemetry.update();

            // State machine
            if (arrived) {
                switch (state) {
                    case TO_START:
                        nav.driveTo(P1[0], P1[1], H1);
                        state = State.MOVE1;
                        break;
                    case MOVE1:
                        nav.driveTo(P2[0], P2[1], H2);
                        state = State.MOVE2;
                        break;
                    case MOVE2:
                        nav.driveTo(P3[0], P3[1], H3);
                        state = State.MOVE3;
                        break;
                    case MOVE3:
                        nav.driveTo(P4[0], P4[1], H4);
                        state = State.MOVE4;
                        break;
                    case MOVE4:
                        nav.driveTo(P5[0], P5[1], H5);
                        state = State.MOVE5;
                        break;
                    case MOVE5:
                        state = State.DONE;
                        break;
                    case DONE:
                    default:
                        hw.leftFront.setPower(0);
                        hw.rightFront.setPower(0);
                        hw.leftRear.setPower(0);
                        hw.rightRear.setPower(0);
                        return;
                }
            }
        }
    }
}
