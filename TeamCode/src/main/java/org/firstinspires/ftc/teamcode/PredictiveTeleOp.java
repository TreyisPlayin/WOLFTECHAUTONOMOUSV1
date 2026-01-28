package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;

@TeleOp(name="Predictive_AutoAim_TeleOp")
public class PredictiveTeleOp extends LinearOpMode {
    private RobotHardware robot = new RobotHardware();
    private DECODEPredictiveIntake intake;
    private ShooterManager shooterManager;
    private LimelightTracker vision;
    private Follower follower;

    private enum Mode { DRIVING, FIRING }
    private Mode currentMode = Mode.DRIVING;
    private int artifactsLaunched = 0;
    private Timer sequenceTimer = new Timer();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        intake = new DECODEPredictiveIntake(hardwareMap);
        shooterManager = new ShooterManager();
        shooterManager.loadTrainingData();
        vision = new LimelightTracker(robot.limelight);

        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();

        waitForStart();

        while (opModeIsActive()) {
            follower.update(); // Compute PedroPathing drive vectors

            // Sync the predictive intake's internal storage logic
            intake.update(0, gamepad1.right_bumper);

            if (currentMode == Mode.DRIVING) {
                handleDriving();
            } else {
                handleFiringSequence();
            }

            telemetry.addData("Mode", currentMode);
            telemetry.addData("Artifacts Launched", artifactsLaunched);
            telemetry.update();
        }
    }

    private void handleDriving() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;

        // --- AUTO-AIM LOGIC (Hold both triggers) ---
        boolean triggersHeld = gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5;
        boolean aligned = false;
        boolean inZone = false;

        if (triggersHeld && vision.hasTarget()) {
            // Overwrite rotation stick with vision alignment PID [1, 2]
            double tx = vision.getTX();
            rotate = -tx * 0.035; // Tuned Proportional constant

            aligned = Math.abs(tx) < 1.5;
            inZone = isInScoringZone(); // Scoring Zone Check

            // Continuous Shooter Prep while aiming
            double dist = vision.getDistance();
            shooterManager.updateServo(robot.hood, shooterManager.getTargetHoodAngle(dist));
            robot.flywheel.setPower(shooterManager.getFlywheelPower(dist, robot.flywheel.getVelocity()));
        }

        // LED Indicator logic: Green only if aligned AND in zone
        if (triggersHeld) {
            robot.setLed((aligned && inZone)? RobotHardware.pwmGreen : RobotHardware.pwmRed);
        } else {
            robot.setLed(RobotHardware.pwmOff);
        }

        // --- TRIGGER RELEASE ACTION ---
        // Release triggers while Green to enter Unstoppable Firing Mode
        if (!triggersHeld && (aligned && inZone) && gamepad1.left_trigger < 0.1) {
            currentMode = Mode.FIRING;
            artifactsLaunched = 0;
            sequenceTimer.resetTimer();
        }

        follower.setTeleOpMovementVectors(drive, strafe, rotate);
    }

    private void handleFiringSequence() {
        // Locked drivetrain - operator cannot stop this
        follower.setTeleOpMovementVectors(0, 0, 0);

        // Transfer Logic: Pull from alternating channels
        // Open the relevant gate based on the current artifact count
        if (artifactsLaunched % 2 == 0) robot.lrPusher.setPosition(DECODEPredictiveIntake.PUSH_L);
        else robot.lrPusher.setPosition(DECODEPredictiveIntake.PUSH_R);

        // Detect successful launch via the touch sensor
        if (robot.launchTrigger.isPressed() && sequenceTimer.getElapsedTimeSeconds() > 0.4) {
            artifactsLaunched++;
            sequenceTimer.resetTimer(); // Wait for next ball to settle into the gate

            if (artifactsLaunched >= 3) {
                currentMode = Mode.DRIVING; // Return control to operator
            }
        }
    }

    /**
     * Determines if the robot is in a legal scoring zone using Botpose.
     * This avoids using PedroPathing's coordinate system if preferred.
     */
    private boolean isInScoringZone() {
        // Retrieve field-relative pose from Limelight MegaTag 2
        com.qualcomm.hardware.limelightvision.LLResult result = robot.limelight.getLatestResult();
        if (result!= null && result.isValid()) {
            double x = result.getBotpose().getPosition().x;
            double y = result.getBotpose().getPosition().y;
            // DECODE Scoring Zone Bounding Box (Adjust for your side)
            return x > 1.2 && y > 1.8; // Example meter coordinates
        }
        return false;
    }
}