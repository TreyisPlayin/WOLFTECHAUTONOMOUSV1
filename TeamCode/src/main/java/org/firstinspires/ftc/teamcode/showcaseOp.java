package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Showcase", group="robot")
public class showcaseOp extends LinearOpMode {

    HardwareConfig robot = new HardwareConfig(this);

    @Override
    public void runOpMode() {
        Pose2D pos = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
        double drive = 0;
        double strafe = 0;
        double turn = 0;
        double altDrive = 0;
        boolean inSettings = false;
        int setting = 1;
        double maxSpeed = 0.5;

        //power variables
        double flPower = 0;
        double frPower = 0;
        double rlPower = 0;
        double rrPower = 0;

        robot.init(hardwareMap, telemetry);

        waitForStart();


        while(opModeIsActive()) {
            robot.pinpoint.update();
            if(gamepad1.start && !inSettings) {
                inSettings = true;
            }
            if(inSettings) {
                telemetry.addData("Settings", " Press a Button to Return");
                telemetry.addData("A", " Rover Drive");
                telemetry.addData("B", " Robot Centric");
                telemetry.addData("X", " Field Centric");
                telemetry.update();
                if(gamepad1.a) {
                    setting = 1;
                    inSettings = false;
                }
                if(gamepad1.b) {
                    setting = 2;
                    inSettings = false;
                }
                if(gamepad1.x) {
                    setting = 3;
                    inSettings = false;
                }
                continue;
            }
            drive = -gamepad1.left_stick_y;
            altDrive = -gamepad1.right_stick_y;
            strafe = gamepad1.left_stick_x;
            turn = gamepad1.right_stick_x;

            if (setting == 1) {
                flPower = drive;
                rlPower = drive;
                frPower = altDrive;
                rrPower = altDrive;
            } else if (setting == 2) {
                flPower = drive + strafe + turn;
                rlPower = drive - strafe + turn;
                frPower = drive - strafe - turn;
                rrPower = drive + strafe - turn;

                double maxPower = 1.0;
                maxPower = Math.max(maxPower, Math.abs(flPower));
                maxPower = Math.max(maxPower, Math.abs(frPower));
                maxPower = Math.max(maxPower, Math.abs(rrPower));
                maxPower = Math.max(maxPower, Math.abs(rlPower));

                flPower /= maxPower;
                frPower /= maxPower;
                rlPower /= maxPower;
                rrPower /= maxPower;

            } else if (setting == 3) {
                double theta = Math.atan2(drive, strafe);
                double r = Math.hypot(strafe, drive);

                theta = AngleUnit.normalizeRadians(theta - robot.pinpoint.getHeading(AngleUnit.RADIANS));
                double newDrive = r * Math.sin(theta);
                double newStrafe = r * Math.cos(theta);

                flPower = newDrive + newStrafe + turn;
                rlPower = newDrive - newStrafe + turn;
                frPower = newDrive - newStrafe - turn;
                rrPower = newDrive + newStrafe - turn;

                double maxPower = 1.0;
                maxPower = Math.max(maxPower, Math.abs(flPower));
                maxPower = Math.max(maxPower, Math.abs(frPower));
                maxPower = Math.max(maxPower, Math.abs(rrPower));
                maxPower = Math.max(maxPower, Math.abs(rlPower));

                flPower /= maxPower;
                frPower /= maxPower;
                rlPower /= maxPower;
                rrPower /= maxPower;

            }


            robot.leftFront.setPower(flPower * maxSpeed);
            robot.rightFront.setPower(frPower * maxSpeed);
            robot.leftRear.setPower(rlPower * maxSpeed);
            robot.rightRear.setPower(rrPower* maxSpeed);

        }
    }




}
