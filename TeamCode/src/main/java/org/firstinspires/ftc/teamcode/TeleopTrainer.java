package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name="Empirical_Shooter_Trainer")
public class TeleopTrainer extends LinearOpMode {
    double speed = 2500, angle = 15;

    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware(); robot.init(hardwareMap);
        ShooterManager shooter = new ShooterManager();
        LimelightTracker vision = new LimelightTracker(robot.limelight);

        waitForStart();
        while (opModeIsActive()) {
            shooter.tunePIDF(ShooterManager.kP, ShooterManager.kI, ShooterManager.kD);
            if (gamepad1.dpad_up) speed += 50; if (gamepad1.dpad_down) speed -= 50;
            if (gamepad1.dpad_right) angle += 1; if (gamepad1.dpad_left) angle -= 1;

            shooter.updateServo(robot.hood, angle);
            robot.flywheel.setVelocity(speed);

            boolean aligned = vision.hasTarget() && Math.abs(vision.getTX()) < 1.5;
            robot.setLed(aligned? RobotHardware.LED_GREEN : RobotHardware.LED_RED);

            if (gamepad1.b |

                    | gamepad1.x) { // B=Hit, X=Miss
                shooter.saveShot(vision.getDistance(), speed, angle, gamepad1.b);
                sleep(500);
            }
            telemetry.addData("Dist", vision.getDistance());
            telemetry.update();
        }
    }
}