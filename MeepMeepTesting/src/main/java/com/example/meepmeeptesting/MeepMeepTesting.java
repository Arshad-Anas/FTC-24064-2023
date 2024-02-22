package com.example.meepmeeptesting;

import static java.lang.Math.nextUp;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;


public class MeepMeepTesting {

    static boolean isRed = true,
            isParkedMiddle = true,
            isUnderTruss = true;

    public static double
            START_X = 12;

    public static double
            BACKBOARD_X = 51.95,
            ANGLE_1 = 41.9,
            ANGLE_2 = 33.45,
            ANGLE_3 = 22,
            ANGLE_4 = 13.8,
            ANGLE_5 = 2;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static EditablePose
            start = new EditablePose(START_X, -61.788975, BACKWARD),
            spikeLeftBlue = new EditablePose((START_X - 6), -34.5, toRadians(135)),
            spikeCenterBlue = new EditablePose((START_X + 6), -26, toRadians(315)),
            spikeRightBlue = new EditablePose(30, -36, toRadians(315)),
            spikeLeftRed = new EditablePose(3, -35, toRadians(135)),
            spikeCenterRed = new EditablePose(24, -27, RIGHT),
            spikeRightRed = new EditablePose(START_X + 14, -34.5, toRadians(315)),
            backboardLeft = new EditablePose(BACKBOARD_X, -30.5, LEFT),
            backboardCenter = new EditablePose(BACKBOARD_X, -34.5, LEFT),
            backboardRight = new EditablePose(BACKBOARD_X, -41, LEFT),
            parkingLeft = new EditablePose(48.5, -10, toRadians(165)),
            parkingRight = new EditablePose(48.5, -56, toRadians(200)),
            spikeDodgeStageDoor = new EditablePose(23, -10, LEFT),
            stageDoor = new EditablePose(13, -10, LEFT),
            innerTruss = new EditablePose(-8, -34.5, LEFT),
            outerTruss = new EditablePose(23.5, -58, LEFT),
            outerTruss2 = new EditablePose(-23.5, -58, LEFT),
            pixelStack1 = new EditablePose(-56.6, -12, LEFT),
            pixelStack3 = new EditablePose(-56.6, -35, LEFT);

    private static EditablePose mainSpike, pixelStack, whiteScoring, yellowScoring, transition;

    public static int randomization = 2;


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(66.10439359219907, 82.63049199024884, 5, toRadians(180),16.08 )
                .setDimensions(16.2981681102, 17.0079)
                .followTrajectorySequence(drive -> {
                    switch (randomization) {
                        case 0:
                            mainSpike = isRed ? spikeLeftRed : spikeRightBlue;
                            yellowScoring = isRed ? backboardLeft : backboardRight;
                            transition = isRed ? (isUnderTruss ? outerTruss : stageDoor) : (isUnderTruss ? outerTruss : spikeDodgeStageDoor);                pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
                            whiteScoring = isUnderTruss ? backboardRight : backboardCenter;
                            break;
                        case 1:
                            mainSpike = isRed ? spikeCenterRed : spikeCenterBlue;
                            yellowScoring = backboardCenter;
                            transition = isRed ? (isUnderTruss ? innerTruss : stageDoor) : (isUnderTruss ? innerTruss : spikeDodgeStageDoor);
                            pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
                            whiteScoring = isUnderTruss ? backboardRight : backboardLeft;
                            break;
                        case 2:
                            mainSpike = isRed ? spikeRightRed : spikeLeftBlue;
                            yellowScoring = isRed ? backboardRight : backboardLeft;
                            transition = isRed ? (isUnderTruss ? outerTruss : spikeDodgeStageDoor) : (isUnderTruss ? outerTruss : stageDoor);
                            pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
                            whiteScoring = isUnderTruss ? backboardRight : backboardLeft;
                            break;
                    }

                    Pose2d startPose = start.byAlliancePose2d();
                    TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);

                    scorePurplePixel(builder, randomization);
                    scoreYellowPixel(builder);
                    getWhitePixels(builder, randomization, 1);
                    scoreWhitePixels(builder, randomization);
                    getWhitePixels(builder, randomization, 2);
                    scoreWhitePixels(builder, randomization);

                    builder.lineToSplineHeading((isParkedMiddle ? parkingLeft : parkingRight).byAlliancePose2d());

                    return builder.build();
                });

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.85f)
                .addEntity(myBot)
                .start();
    }


    private static void scorePurplePixel(TrajectorySequenceBuilder builder, int randomization) {

        if (isBackboardSide(randomization) || randomization == 1) {
            builder.lineToSplineHeading(mainSpike.byAlliancePose2d());
        } else {
            builder.setTangent(isRed ? FORWARD : BACKWARD)
                    .splineTo(mainSpike.byAllianceVec(), mainSpike.byAlliance().heading);
        }

    }

    private static void scoreYellowPixel(TrajectorySequenceBuilder builder) {
        builder.lineToSplineHeading(yellowScoring.byAlliancePose2d());
    }

    private static void getWhitePixels(TrajectorySequenceBuilder builder, int randomization, int cycle) {
        builder.setTangent(LEFT)
                .splineTo(transition.byAllianceVec(), transition.heading);

        if (isUnderTruss && randomization != 1) builder.splineTo(outerTruss2.byAllianceVec(), outerTruss2.heading);

        builder.splineTo(pixelStack.byAllianceVec(), pixelStack.heading);
    }

    private static void scoreWhitePixels(TrajectorySequenceBuilder builder, int randomization) {
        builder.setTangent(RIGHT);

        if (isUnderTruss && randomization != 1) builder.splineTo(outerTruss2.byAllianceVec(), RIGHT);

        builder.splineTo(transition.byAllianceVec(), RIGHT)
                .splineTo(whiteScoring.byAllianceVec(), RIGHT);

    }

    private static boolean isCenter(int randomization) {
        return randomization == 1;
    }

    private static boolean isAudienceSide(int randomization) {
        return isRed && randomization == 0 || !isRed && randomization == 2;
    }

    private static boolean isBackboardSide(int randomization) {
        return isRed && randomization == 2 || !isRed && randomization == 0;
    }
    
    private static class EditablePose {
        public double x, y, heading;

        private EditablePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        // Switches *red* alliance coordinates to blue alliance
        private EditablePose byAlliance() {
            if (!isRed) {
                y *= -1;
                heading *= -1;
            }
            return this;
        }

        private Pose2d byAlliancePose2d() {
            return byAlliance().toPose2d();
        }

        private Vector2d byAllianceVec() {
            return byAlliancePose2d().vec();
        }

        private Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }

//        private EditablePose bySide() {
//            if (isTop != isRed) x += (X_START_BOTTOM - X_START_TOP);
//            return this;
//        }
//
//        private EditablePose byBoth() {
//            return byAlliance().bySide();
//        }
//
//        private EditablePose flipBySide() {
//            boolean isRight = MainAuton.isTop == isRed;
//            if (!isTop) heading = Math.PI - heading;
//            if (!isTop) x = (X_START_BOTTOM + X_START_TOP) / 2 - x;
//            return this;
//        }
    }
}