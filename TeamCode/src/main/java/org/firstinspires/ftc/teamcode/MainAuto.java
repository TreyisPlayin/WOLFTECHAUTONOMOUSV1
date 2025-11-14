package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Main autonomous without PresetManager / AutonomousRunner:
 *
 *  - Uses Odometry + AprilTagLocalizer for localization.
 *  - In init: "finds its tile" by snapping odometry pose to an AprilTag pose if seen.
 *  - After start:
 *      1) Drive to FAR scoring position.
 *      2) Align to goal + fire LEFT catapult + reload (3 times total).
 *      3) Drive to pickup area and let HuskyLens + servo push grab an artifact.
 *
 * TUNE: all the coordinate constants and timings near the top.
 */
@Autonomous(name = "MainAuto", group = "Auto")
public class MainAuto extends LinearOpMode {

    // ==== FIELD CONSTANTS (inches, 0–143 map) – TUNE THESE ====

    //  Start tile center + starting heading
    private static final double START_X_IN  = 18.0;
    private static final double START_Y_IN  = 18.0;
    private static final double START_H_DEG =   0.0; // 0° = +X direction

    //  Far-away scoring pose (where you want to sit & shoot from)
    private static final double FAR_X_IN        = 120.0;
    private static final double FAR_Y_IN        = 110.0;
    private static final double FAR_HEADING_DEG =   0.0;

    //  Pickup area (where you want to go hunt for artifacts after 3 shots)
    private static final double PICKUP_X_IN        = 72.0;
    private static final double PICKUP_Y_IN        = 72.0;
    private static final double PICKUP_HEADING_DEG = 180.0;

    // === Drive tuning ===
    private static final double DRIVE_KP_POS   = 0.02;  // position P gain
    private static final double DRIVE_KP_HEAD  = 0.01;  // heading P gain
    private static final double MAX_DRIVE_PWR  = 0.7;
    private static final double MAX_TURN_PWR   = 0.5;
    private static final double POS_TOL_IN     = 1.5;   // distance tolerance
    private static final double HEAD_TOL_DEG   = 3.0;   // heading tolerance

    // === Catapult & reload timing (TUNE THESE) ===
    private static final long CATAPULT_FIRE_MS   = 350;
    private static final long RELOAD_EXTEND_MS   = 300;
    private static final long RELOAD_RETRACT_MS  = 200;

    // === HuskyLens pickup tuning ===
    private static final int   HUSKY_PIXEL_TOL       = 20;
    private static final double HUSKY_TARGET_DIST_IN = 6.0;

    // === Auto state machine ===
    private enum State {
        DRIVE_TO_FAR,
        ALIGN_AND_FIRE,
        RELOAD,
        DECIDE_NEXT_SHOT,
        DRIVE_TO_PICKUP,
        PICKUP,
        DONE
    }

    @Override
    public void runOpMode() {

        // ---- Hardware + subsystems ----
        HardwareConfig robot      = new HardwareConfig(hardwareMap);
        Odometry      odo        = new Odometry(robot);
        IntakeSystem  intake     = new IntakeSystem(robot);
        VisionWebcamPortal portal = new VisionWebcamPortal(robot);
        HuskyLensStereo husky     = new HuskyLensStereo(
                robot,
                /*targetID*/ 1,    // TODO: set this to your HuskyLens trained ID for artifacts
                /*A baseline*/ 5.0,
                /*FOV W*/    60.0,
                /*imgWidth*/ 320
        );

        // AprilTag localization (this is your "AprilTag localization class")
        AprilTagLocalizer tagLoc = new AprilTagLocalizer(
                portal.april,
                new AprilTagLocalizer.CamToRobot(
                        /*cam X offset from robot center*/ 5.0,
                        /*cam Y offset*/                    0.0,
                        /*cam heading offset*/              0.0
                )
        );

        // ---- Configure AprilTag field layout (EXAMPLE – REPLACE with real game layout) ----
        // id, xIn, yIn, headingDeg
        tagLoc.addFieldTag(1,  12, 132, 180);
        tagLoc.addFieldTag(2, 132, 132, -90);
        tagLoc.addFieldTag(3, 132,  12,   0);
        tagLoc.addFieldTag(4,  12,  12,  90);

        // ---- Initial pose guess: center of starting tile ----
        odo.setPose(new Position(START_X_IN, START_Y_IN, START_H_DEG));

        telemetry.addLine("MainAuto: INIT — looking for AprilTags to lock start pose...");
        telemetry.update();

        // ==== INIT-LOOP: "find tile" via AprilTags before the match starts ====
        Position lockedPose = null;
        while (!isStarted() && !isStopRequested()) {
            odo.update();   // deadwheel pose
            Position fused = tagLoc.maybeFuseWithOdometry(odo); // snap to tag if seen

            if (fused != null) {
                lockedPose = fused;
                telemetry.addData("Start pose (tag fused)", "%s", fused);
            } else {
                Position guess = odo.getCurrentPosition();
                telemetry.addData("Start pose (tile guess)", "%s", guess);
            }

            telemetry.update();
            sleep(50);
        }

        if (isStopRequested()) return;

        telemetry.addLine("MainAuto: START!");
        if (lockedPose != null) telemetry.addData("Locked pose", "%s", lockedPose);
        telemetry.update();

        // ==== RUN AUTO ====
        State state = State.DRIVE_TO_FAR;
        int shotsFired = 0;
        ElapsedTime timer = new ElapsedTime();

        while (opModeIsActive() && state != State.DONE) {

            // 1) Update localization: odometry + AprilTag fusion
            odo.update();
            tagLoc.maybeFuseWithOdometry(odo);

            Position pose = odo.getCurrentPosition();

            telemetry.addData("State", state);
            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f",
                    pose.getX(), pose.getY(), pose.getHeading());
            telemetry.addData("Shots fired", shotsFired);

            switch (state) {

                // ------------------------------------------------------------
                case DRIVE_TO_FAR: {
                    boolean atTarget = driveToPoint(
                            robot,
                            pose,
                            FAR_X_IN,
                            FAR_Y_IN,
                            FAR_HEADING_DEG
                    );
                    if (atTarget) {
                        stopDrive(robot);
                        state = State.ALIGN_AND_FIRE;
                        timer.reset();
                    }
                    break;
                }

                // ------------------------------------------------------------
                case ALIGN_AND_FIRE: {
                    // Rotate in place to face the goal using pose
                    boolean aligned = alignToGoalHeading(robot, pose, FAR_X_IN, FAR_Y_IN);
                    if (!aligned) break;

                    // Once aligned, FIRE LEFT CATAPULT
                    if (robot.leftCatapult != null) {
                        robot.leftCatapult.setPower(1.0);
                        sleep(CATAPULT_FIRE_MS);
                        robot.leftCatapult.setPower(0.0);
                    }
                    shotsFired++;
                    state = State.RELOAD;
                    timer.reset();
                    break;
                }

                // ------------------------------------------------------------
                case RELOAD: {
                    // Use a reload servo to feed the next purple into the left catapult.
                    // Requires: public Servo catapultLoader; in HardwareConfig.
                    if (robot.catapultLoader != null) {
                        robot.catapultLoader.setPosition(1.0); // extend
                        sleep(RELOAD_EXTEND_MS);
                        robot.catapultLoader.setPosition(0.0); // retract
                        sleep(RELOAD_RETRACT_MS);
                    }
                    state = State.DECIDE_NEXT_SHOT;
                    break;
                }

                // ------------------------------------------------------------
                case DECIDE_NEXT_SHOT: {
                    if (shotsFired < 3) {
                        // Move again to far spot (in case we drifted) and shoot again
                        state = State.DRIVE_TO_FAR;
                    } else {
                        // After 3 shots, head to pickup area
                        state = State.DRIVE_TO_PICKUP;
                    }
                    break;
                }

                // ------------------------------------------------------------
                case DRIVE_TO_PICKUP: {
                    boolean atPickup = driveToPoint(
                            robot,
                            pose,
                            PICKUP_X_IN,
                            PICKUP_Y_IN,
                            PICKUP_HEADING_DEG
                    );
                    if (atPickup) {
                        stopDrive(robot);
                        state = State.PICKUP;
                        timer.reset();
                    }
                    break;
                }

                // ------------------------------------------------------------
                case PICKUP: {
                    // 1) Let HuskyLens center on the artifact and approach
                    husky.alignToTarget(HUSKY_PIXEL_TOL, HUSKY_TARGET_DIST_IN);

                    // 2) Spin intake and shove artifact in with intakeArm or a pushServo
                    robot.intakeMotor.setPower(1.0);
                    sleep(350);
                    robot.intakeMotor.setPower(0.0);

                    if (robot.intakeArm != null) {
                        robot.intakeArm.setPosition(1.0);
                        sleep(250);
                        robot.intakeArm.setPosition(0.0);
                        sleep(150);
                    }

                    state = State.DONE; // After one pickup sequence; you can expand later
                    break;
                }

                // ------------------------------------------------------------
                case DONE: {
                    stopDrive(robot);
                    break;
                }
            }

            telemetry.update();
        }

        // Safety: stop everything at the end
        stopDrive(robot);
        if (robot.leftCatapult != null) robot.leftCatapult.setPower(0.0);
        robot.intakeMotor.setPower(0.0);
    }

    // ==== Helper: drive toward a target point with field-centric mecanum ====
    private boolean driveToPoint(HardwareConfig robot,
                                 Position pose,
                                 double targetX,
                                 double targetY,
                                 double targetHeadingDeg) {

        double x = pose.getX();
        double y = pose.getY();
        double hDeg = pose.getHeading();

        double dx = targetX - x;
        double dy = targetY - y;
        double dist = Math.hypot(dx, dy);

        double desiredHeading = Math.toDegrees(Math.atan2(dy, dx));
        double headingErrorToPoint = angleWrap(desiredHeading - hDeg);
        double headingErrorFinal   = angleWrap(targetHeadingDeg - hDeg);

        // "Drive" vector in field frame
        double driveMag = DRIVE_KP_POS * dist;
        driveMag = clamp(driveMag, -MAX_DRIVE_PWR, MAX_DRIVE_PWR);

        double dirRad = Math.toRadians(desiredHeading);
        double vxField = driveMag * Math.cos(dirRad); // X in field
        double vyField = driveMag * Math.sin(dirRad); // Y in field

        // Convert field vector to robot frame (for mecanum)
        double hRad = Math.toRadians(hDeg);
        double forward =  Math.cos(hRad)*vxField + Math.sin(hRad)*vyField;
        double strafe  = -Math.sin(hRad)*vxField + Math.cos(hRad)*vyField;

        // Turn toward targetHeading while we move
        double turn = DRIVE_KP_HEAD * headingErrorFinal;
        turn = clamp(turn, -MAX_TURN_PWR, MAX_TURN_PWR);

        // If we're close in position, stop translation and just fix heading
        if (dist < POS_TOL_IN) {
            forward = 0;
            strafe  = 0;
            turn = DRIVE_KP_HEAD * headingErrorFinal;
        }

        setMecanumDrive(robot, strafe, forward, turn);

        // Done if close in both position and heading
        return (dist < POS_TOL_IN) && (Math.abs(headingErrorFinal) < HEAD_TOL_DEG);
    }

    // ==== Helper: align robot to face the FAR goal (for scoring) ====
    private boolean alignToGoalHeading(HardwareConfig robot,
                                       Position pose,
                                       double goalX,
                                       double goalY) {
        double x = pose.getX();
        double y = pose.getY();
        double hDeg = pose.getHeading();

        double desired = Math.toDegrees(Math.atan2(goalY - y, goalX - x));
        double err = angleWrap(desired - hDeg);

        double turn = DRIVE_KP_HEAD * err;
        turn = clamp(turn, -MAX_TURN_PWR, MAX_TURN_PWR);

        // Only rotate in place for fine aiming
        setMecanumDrive(robot, 0, 0, turn);

        return Math.abs(err) < HEAD_TOL_DEG;
    }

    // ==== Mecanum drive helper ====
    private void setMecanumDrive(HardwareConfig robot,
                                 double strafe,
                                 double forward,
                                 double turn) {

        double lf = forward + strafe + turn;
        double rf = forward - strafe - turn;
        double lb = forward - strafe + turn;
        double rb = forward + strafe - turn;

        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lb), Math.abs(rb)))));

        lf /= max;
        rf /= max;
        lb /= max;
        rb /= max;

        robot.leftFront .setPower(lf);
        robot.rightFront.setPower(rf);
        robot.leftBack  .setPower(lb);
        robot.rightBack .setPower(rb);
    }

    private void stopDrive(HardwareConfig robot) {
        setMecanumDrive(robot, 0, 0, 0);
    }

    // ==== Small math helpers ====
    private double angleWrap(double deg) {
        deg = (deg + 540.0) % 360.0 - 180.0;
        return deg;
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
