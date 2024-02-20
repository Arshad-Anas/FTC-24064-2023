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
            X_START_BOTTOM = -37;

    public static double
            BACKBOARD_X = 51.95,
            ANGLE_1 = 41.9,
            ANGLE_2 = 33.45,
            ANGLE_3 = 22,
            ANGLE_4 = 13.8,
            ANGLE_5 = 0;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static EditablePose
            botStartRed = new EditablePose(X_START_BOTTOM, -61.788975, BACKWARD),
            botLeftSpikeRed = new EditablePose(-49 , -16, LEFT),
            botLeftSpikeRed2 = new EditablePose(-56,-36, toRadians(210)),
            botCenterSpikeRed = new EditablePose(-50, -22, LEFT),
            botRightSpikeRed = new EditablePose(-33, -35, toRadians(210)),
            botRightSpikeBlue = new EditablePose(-33,-36, toRadians(170)),
            botCenterSpikeBlue = new EditablePose(-50 , -25, LEFT),
            botCenterBackdropRed = new EditablePose(BACKBOARD_X, -34.5, LEFT),
            botLeftBackdropRed = new EditablePose(BACKBOARD_X, -30.5, LEFT),
            botRightBackdropRed = new EditablePose(BACKBOARD_X, -41, LEFT),
            botParkingLeftRed = new EditablePose(48, -10, LEFT),
            botParkingRightRed = new EditablePose(48, -60, LEFT),
            botAudienceSpikeTransitionRed = new EditablePose(-34,-18,toRadians(110)),
            botStageDoor = new EditablePose(25,-10,LEFT),
            botTrussInner = new EditablePose(20,-36,LEFT),
            botTrussOuter = new EditablePose(20,-58,LEFT),
            firstWhitePixelStackRed = new EditablePose(-56.6,-12, LEFT),
            trussTransition = new EditablePose(-53,-58,LEFT),
            thirdWhitePixelStackRed = new EditablePose(-56.6, -35,LEFT);

    private static EditablePose mainSpike, pixelStack, whiteScoring, yellowScoring, transition;

    public static int randomization = 0;


    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(66.10439359219907, 82.63049199024884, 5, toRadians(180),16.08 )
                .setDimensions(16.2981681102, 17.0079)
                .followTrajectorySequence(drive -> {
                    switch (randomization) {
                        case 0:
                            mainSpike = isRed ?  botLeftSpikeRed : botRightSpikeBlue;
                            yellowScoring = isRed ? botLeftBackdropRed : botRightBackdropRed;
                            transition = isUnderTruss ? botTrussOuter : botStageDoor;
                            pixelStack = isUnderTruss ? thirdWhitePixelStackRed : firstWhitePixelStackRed;
                            whiteScoring = isRed ? botLeftBackdropRed : botRightBackdropRed;
                            break;
                        case 1:
                            mainSpike = isRed ? botCenterSpikeRed: botCenterSpikeBlue;
                            yellowScoring = botCenterBackdropRed;
                            transition = isUnderTruss ? botTrussInner : botStageDoor;
                            pixelStack = isUnderTruss ? thirdWhitePixelStackRed : firstWhitePixelStackRed;
                            whiteScoring = isRed ? botLeftBackdropRed : botRightBackdropRed;
                            break;
                        case 2:
                            mainSpike = isRed ? botRightSpikeRed : botLeftSpikeRed;
                            yellowScoring = isRed ? botRightBackdropRed : botLeftBackdropRed;
                            transition = isUnderTruss ? botTrussOuter : botStageDoor;
                            pixelStack = isUnderTruss ? thirdWhitePixelStackRed : firstWhitePixelStackRed;
                            whiteScoring = isRed ? botLeftBackdropRed : botRightBackdropRed;
                            break;
                    }

                    Pose2d startPose = botStartRed.byAlliancePose2d();
                    TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);

                    scorePurplePixel(builder, randomization); // good
                    getFirstWhitePixel(builder, randomization); // good
                    scoreYellowPixel(builder); // good
                    getWhitePixels(builder, randomization ,1);
                    scoreWhitePixels(builder, randomization);
                    getWhitePixels(builder, randomization, 2);
                    scoreWhitePixels(builder, randomization);

                    return builder.build();
                });

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.85f)
                .addEntity(myBot)
                .start();
    }

    private static void scorePurplePixel(TrajectorySequenceBuilder builder, int randomization) {
        builder.setTangent(isRed ? BACKWARD : FORWARD);
        if (isAudienceSide(randomization) && !isUnderTruss) {
            builder
                    .lineToSplineHeading(botAudienceSpikeTransitionRed.byAlliancePose2d())
                    .setTangent(botAudienceSpikeTransitionRed.heading)
                    .lineTo(mainSpike.byAllianceVec())
                    .setTangent(LEFT);
        } else if (isAudienceSide(randomization) && isUnderTruss) {
            builder
                    .strafeRight(6)
                    .lineToSplineHeading(botLeftSpikeRed2.byAlliancePose2d());
        } else if (isCenter(randomization) || isBackboardSide(randomization)) {
            builder.lineToSplineHeading(mainSpike.byAlliancePose2d());
        }
    }

    private static void getFirstWhitePixel(TrajectorySequenceBuilder builder, int randomization) {
        builder.lineToSplineHeading(pixelStack.byAlliancePose2d());

        if (isUnderTruss && !isCenter(randomization)) builder.lineToConstantHeading(trussTransition.byAllianceVec());

    }
    private static void scoreYellowPixel(TrajectorySequenceBuilder builder) {
        builder
                .setTangent(RIGHT)
                .splineTo(transition.byAllianceVec(), RIGHT)
                .splineToConstantHeading(yellowScoring.byAllianceVec(), RIGHT);
    }
    private static void getWhitePixels(TrajectorySequenceBuilder builder, int randomization, int cycle) {
        builder.setTangent(LEFT)
                .splineToConstantHeading(transition.byAllianceVec(), LEFT);

        if (isUnderTruss && randomization != 1)
            builder.splineToConstantHeading(trussTransition.byAllianceVec(),LEFT)
                    .lineToConstantHeading(pixelStack.byAllianceVec());
        else builder.splineTo(pixelStack.byAllianceVec(), pixelStack.heading);

    }
    private static void scoreWhitePixels(TrajectorySequenceBuilder builder, int randomization) {
    if (isUnderTruss && randomization != 1)
        builder.lineToConstantHeading(trussTransition.byAllianceVec());
    builder.setTangent(RIGHT);

//        if (isUnderTruss && randomization != 1) builder.splineTo(outerTruss2.byAllianceVec(), RIGHT);

    builder.splineTo(transition.byAllianceVec(), RIGHT)
            .splineToConstantHeading(whiteScoring.byAllianceVec(), RIGHT);
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