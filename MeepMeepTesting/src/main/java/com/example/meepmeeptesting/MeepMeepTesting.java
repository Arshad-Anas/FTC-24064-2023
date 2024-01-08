package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    static boolean isRed = false, isParkedLeft = true, isRightCenterSpike = false;

    static final boolean isRight = true;

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12,
            DIRECTION = toRadians(0);

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static int spikeNum = 2;

    public static EditablePose
            startPoseRed = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            startPoseBlue = new EditablePose(startPoseRed.byAlliance().toPose2d().vec().getX(), 61, BACKWARD),
            centerSpikeRed = new EditablePose((X_START_RIGHT + 3.5), -33.5, FORWARD),
            centerSpikeBlue = new EditablePose(centerSpikeRed.byAlliance().toPose2d().vec().getX(), 33.5, BACKWARD),
            leftSpikeRed = new EditablePose(7, -41, toRadians(120)),
            leftSpikeBlue = new EditablePose(leftSpikeRed.byAlliance().toPose2d().vec().getX(), 41, toRadians(-120)),
            rightSpikeRed = new EditablePose(24 - leftSpikeRed.x, leftSpikeRed.y, LEFT - leftSpikeRed.heading),
            rightSpikeBlue = new EditablePose(rightSpikeRed.byAlliance().toPose2d().vec().getX(), 41, LEFT + leftSpikeRed.heading),
            redBackboard = new EditablePose(48, -34, RIGHT),
            blueBackboard = new EditablePose(redBackboard.byAlliance().toPose2d().vec().getX(), 34, RIGHT),
            redParkingLeft = new EditablePose(52, -14, toRadians(165)),
            redParkingRight = new EditablePose(51, -54, toRadians(200)),
            blueParkingLeft = new EditablePose(redParkingLeft.toPose2d().vec().getX(), 60, toRadians(165)),
            blueParkingRight = new EditablePose(redParkingRight.toPose2d().vec().getX(), 14, toRadians(200));

    public static Pose2d
            mainSpikeBlue = null,
            mainSpikeRed = null;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double
                OUTTAKE_WAIT_TIME = 0.25,
                INTAKE_WAIT_TIME = 0.65,
                SCORING_WAIT_TIME = 0.75,
                OPEN_FLAP_WAIT_TIME = 0.25;

        Pose2d startPoseBlue = MeepMeepTesting.startPoseBlue.toPose2d();
        Pose2d startPoseRed = MeepMeepTesting.startPoseRed.byBoth().toPose2d();
        Pose2d centerSpikeBlue = MeepMeepTesting.centerSpikeBlue.toPose2d();
        Pose2d centerSpikeRed = MeepMeepTesting.centerSpikeRed.byBoth().toPose2d();
        Pose2d leftSpikeBlue = MeepMeepTesting.leftSpikeBlue.toPose2d();
        Pose2d leftSpikeRed = MeepMeepTesting.leftSpikeRed.byBoth().toPose2d();
        Pose2d rightSpikeBlue = MeepMeepTesting.rightSpikeBlue.toPose2d();
        Pose2d rightSpikeRed = MeepMeepTesting.rightSpikeRed.byBoth().toPose2d();
        Pose2d blueBackboard = MeepMeepTesting.blueBackboard.toPose2d();
        Pose2d redBackboard = MeepMeepTesting.redBackboard.byBoth().toPose2d();
        Pose2d redParkingLeft = MeepMeepTesting.redParkingLeft.byBoth().toPose2d();
        Pose2d redParkingRight = MeepMeepTesting.redParkingRight.byBoth().toPose2d();
        Pose2d blueParkingLeft = MeepMeepTesting.blueParkingLeft.toPose2d();
        Pose2d blueParkingRight = MeepMeepTesting.blueParkingRight.toPose2d();

        switch (spikeNum) {
            case 0: {
                mainSpikeBlue = leftSpikeBlue;
                mainSpikeRed = leftSpikeRed;
                break;
            }

            case 1: {
                mainSpikeBlue = centerSpikeBlue;
                mainSpikeRed = centerSpikeRed;
                break;
            }

            case 2: {
                mainSpikeBlue = rightSpikeBlue;
                mainSpikeRed = rightSpikeRed;
                isRightCenterSpike = true;
                break;
            }
        }

        switch (spikeNum) {
            case 0: {
                DIRECTION = (isRed ? toRadians(135) : toRadians(-135));
                break;
            }

            case 1: {
                DIRECTION = (isRed ? FORWARD : BACKWARD);
                break;
            }

            case 2: {
                DIRECTION = (isRed ? (LEFT - toRadians(135)) : (LEFT - toRadians(-135)));
                break;
            }
        }

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, toRadians(60), toRadians(60), 16.02362205)
                .setDimensions(16.2981681102, 17.0079)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(isRed ? startPoseRed : startPoseBlue)
                                .splineTo(isRed ? mainSpikeRed.vec() : mainSpikeBlue.vec(), DIRECTION)
                                /*
                                   Starts outtake 0.5 seconds after prev. action, then waits 0.25 seconds before stopping the outtake
                                   Then stops after 1 second
                                 */
                                // .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intake.set(0.35))
                                // .addTemporalMarker(0.5 + OUTTAKE_WAIT_TIME, () -> intake.set(0))
                                .strafeRight(isRightCenterSpike ? (isRed ? 8 : -8) : 0.0001)
                                .turn(isRightCenterSpike ? (isRed ? (RIGHT - toRadians(35)) : (RIGHT + toRadians(35))) : 0)
                                .lineToSplineHeading(isRed ? redBackboard : blueBackboard)
                                /*
                                    Do april tag stuff here because now we can scan
                                 */
                                .turn(LEFT)
                                /*
                                 Starts the lift by updating target to row 0, then executes commands to do so (within timing)
                                 It will activate flap to open, releasing the pixels
                                 After doing that, it'll retract back to target row -1
                                */
                                // .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                //      robot.lift.setTargetRow(0);
                                //      robot.lift.updateTarget();
                                // })
                                // .addTemporalMarker(0.5 + OPEN_FLAP_WAIT_TIME, () -> robot.arm.setFlap(true))
                                // .addTemporalMarker((0.5 + OPEN_FLAP_WAIT_TIME) + SCORING_WAIT_TIME, () -> robot.arm.setArm(true))
                                .lineToSplineHeading(isRed ? (isParkedLeft ? redParkingLeft : redParkingRight) : (isParkedLeft ? blueParkingLeft : blueParkingRight))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.85f)
                .addEntity(myBot)
                .start();
    }

    private static class EditablePose {

        public double x, y, heading;

        private EditablePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        private EditablePose byAlliance() {
            if (!isRed) y = -1;
            if (!isRed) heading= -1;
            return this;
        }

        private EditablePose bySide() {
            if (isRight != isRed) x += (X_START_LEFT - X_START_RIGHT);
            return this;
        }

        private EditablePose byBoth() {
            return byAlliance().bySide();
        }

        private EditablePose flipBySide() {
            boolean isRight = MeepMeepTesting.isRight == isRed;
            if (!isRight) heading = PI - heading;
            if (!isRight) x = (X_START_LEFT + X_START_RIGHT) / 2 - x;
            return this;
        }

        private Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }
    }
}