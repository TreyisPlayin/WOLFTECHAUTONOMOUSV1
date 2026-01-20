package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="3_Ball_Cycle_Auto")
public class HighScoreAuto extends LinearOpMode {
    private RobotHardware robot = new RobotHardware();
    private Follower follower;
    private ShooterManager shooter;
    private LimelightTracker vision;

    private enum State { INTAKE, SHOOT, PARK, DONE }
    private State currentState = State.INTAKE;
    private int ballCount = 0;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(28.5, 128, Math.toRadians(180)));
        shooter = new ShooterManager(); shooter.loadTrainingData();
        vision = new LimelightTracker(robot.limelight);

        waitForStart();
        while (opModeIsActive()) {
            follower.update(); // Compute wheel powers

            switch (currentState) {
                case INTAKE:
                    follower.followPath(new BezierLine(follower.getPose(), new Pose(37, 121, 0)));
                    robot.intakeL.setPower(1.0);

                    // Count 3 balls using dual distance sensors
                    if (robot.distL.getDistance(DistanceUnit.CM) < 5.0 |

                            | robot.distR.getDistance(DistanceUnit.CM) < 5.0) {
                        ballCount++; sleep(400);
                    }
                    if (ballCount >= 3) { // AUTO-SWITCH TRIGGER
                        robot.intakeL.setPower(0); currentState = State.SHOOT;
                    }
                    break;

                case SHOOT:
                    follower.followPath(new BezierLine(follower.getPose(), new Pose(60, 85, Math.toRadians(135))));
                    if (vision.hasTarget()) {
                        double d = vision.getDistance();
                        shooter.updateServo(robot.hood, shooter.hoodTable.get(d));
                        robot.flywheel.setPower(shooter.getFlywheelPower(d, robot.flywheel.getVelocity()));

                        if (!follower.isBusy() && Math.abs(vision.getTX()) < 1.5) {
                            fireArtifacts(); // Uses launchTrigger to decrement ballCount
                            if (ballCount <= 0) currentState = State.INTAKE;
                        }
                    }
                    break;

                case PARK:
                    follower.followPath(new BezierLine(follower.getPose(), new Pose(72, 72, Math.toRadians(90))));
                    currentState = State.DONE;
                    break;
            }
        }
    }

    private void fireArtifacts() {
        robot.leftGate.setPosition(1.0); robot.pusher.setPower(0.8);
        if (robot.launchTrigger.isPressed()) {
            robot.leftGate.setPosition(0.0); ballCount--;
        }
    }
}