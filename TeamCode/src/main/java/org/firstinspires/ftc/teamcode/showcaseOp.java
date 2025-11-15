package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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
        double targetDistanceIn = 36.0;
        boolean prevRightBumper = false;
        boolean prevDpadUp = false;
        boolean prevDpadDown = false;
        boolean prevDpadLeft = false;
        boolean prevDpadRight = false;

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

            if (gamepad1.dpad_up && !prevDpadUp) {
                targetDistanceIn = Math.min(targetDistanceIn + 1.0, 96.0);
            }
            if (gamepad1.dpad_down && !prevDpadDown) {
                targetDistanceIn = Math.max(targetDistanceIn - 1.0, 12.0);
            }
            if (gamepad1.dpad_right && !prevDpadRight) {
                targetDistanceIn = Math.min(targetDistanceIn + 6.0, 120.0);
            }
            if (gamepad1.dpad_left && !prevDpadLeft) {
                targetDistanceIn = Math.max(targetDistanceIn - 6.0, 12.0);
            }

            ShooterModel.ShotProfile shot = ShooterModel.calculateShot(targetDistanceIn);

            boolean launcherAvailable = robot.leftCatapult != null || robot.rightCatapult != null;
            if (gamepad1.right_bumper && !prevRightBumper && launcherAvailable) {
                fireBothCatapults(shot.power, 800);
            }

            prevRightBumper = gamepad1.right_bumper;
            prevDpadUp = gamepad1.dpad_up;
            prevDpadDown = gamepad1.dpad_down;
            prevDpadLeft = gamepad1.dpad_left;
            prevDpadRight = gamepad1.dpad_right;

            if (robot.intakeMotor != null) {
                double intakePower = ShooterModel.clamp(gamepad1.right_trigger - gamepad1.left_trigger, -1.0, 1.0);
                robot.intakeMotor.setPower(intakePower);
                telemetry.addData("Intake power", "%.2f", intakePower);
            }

            telemetry.addData("Shot distance (in)", "%.1f", targetDistanceIn);
            telemetry.addData("Shot power", "%.2f", shot.power);
            telemetry.addData("Required RPM", "%.0f", shot.requiredRpm);
            telemetry.addData("Launcher ready", launcherAvailable);
            if (!launcherAvailable) {
                telemetry.addLine("Configure catapult motors in the Robot Controller to enable launching.");
            }
            telemetry.addData("Launch angle (deg)", "%.1f", ShooterModel.LAUNCH_ANGLE_DEG);
            telemetry.update();

        }
    }




    private void fireBothCatapults(double power, long durationMs) {
        DcMotor left  = robot.leftCatapult;
        DcMotor right = robot.rightCatapult;

        power = ShooterModel.clamp(power, -1.0, 1.0);

        if (left != null) {
            left.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            left.setPower(power);
        }
        if (right != null) {
            right.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            right.setPower(power);
        }

        sleep(durationMs);

        if (left != null)  left.setPower(0.0);
        if (right != null) right.setPower(0.0);
    }
}
