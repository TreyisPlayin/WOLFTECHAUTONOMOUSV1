package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import java.util.List;

/**
 * AutonomousStateMachine.java
 *
 * The main OpMode: cycles through STARTING→PICKUP→SCORE states,
 * using PathPlanner, FieldMapper, Odometry, AprilTagTracker, CameraSync, and IntakeSystem.
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name="AutonomousW/States")
public class Autonomous extends LinearOpMode {
    private HardwareConfig    robot;
    private FieldMapper       fieldMapper;
    private PathPlanner       planner;
    private Odometry          odometry;
    private AprilTagTracker   tagTracker;
    private CameraSync        camSync;
    private IntakeSystem      intake;

    private enum State {STARTING, PICKUP, SCORE, HUMAN_PLAYER}
    private State state = State.STARTING;
    private static final int TARGET_ID = 1;

    @Override
    public void runOpMode() {
        robot      = new HardwareConfig(hardwareMap);
        fieldMapper= new FieldMapper(ZoneMap.getRedZoneMap());
        planner    = new PathPlanner();
        odometry   = new Odometry(robot);
        tagTracker = new AprilTagTracker(robot, odometry);
        camSync    = new CameraSync();
        intake     = new IntakeSystem(robot);

        waitForStart();
        while (opModeIsActive()) {
            odometry.update();
            tagTracker.checkForTags();
            fieldMapper.scanWithUltrasonics(robot, odometry);

            switch (state) {
                case STARTING:
                    goTo(fieldMapper.getStartingZone());
                    state = State.PICKUP;
                    break;

                case PICKUP:
                    if (camSync.isSameObjectVisible(robot.leftCam, robot.rightCam, TARGET_ID)) {
                        camSync.alignToObject(robot, robot.leftCam, robot.rightCam, TARGET_ID);
                        intake.autoGrab();
                        state = State.SCORE;
                    } else {
                        goTo(fieldMapper.getCollectionZone());
                    }
                    break;

                case SCORE:
                    goTo(fieldMapper.getScoringZone());
                    intake.release();
                    state = State.PICKUP;
                    break;

                case HUMAN_PLAYER:
                    goTo(fieldMapper.getHumanPlayerZone());
                    // your custom logic...
                    state = State.SCORE;
                    break;
            }
        }
    }

    private void goTo(Position dest) {
        List<Position> path = planner.aStar(fieldMapper.getFieldGrid(),
                odometry.getCurrentPosition(), dest);
        planner.followPath(path, robot, odometry);
    }
}
