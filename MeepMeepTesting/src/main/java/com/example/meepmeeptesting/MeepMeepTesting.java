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

    static boolean isRed = false,
            isParkedMiddle = false,
            isUnderTruss = false;

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
            spikeCenterBlue = new EditablePose((START_X + 12), -22, toRadians(0)),
            spikeRightBlue = new EditablePose(30, -36, toRadians(315)),
            spikeLeftRed = new EditablePose(3, -35, toRadians(135)),
            spikeCenterRed = new EditablePose(24, -27, RIGHT),
            spikeRightRed = new EditablePose(START_X + 14, -40, toRadians(315)),
            backboardLeft = new EditablePose(BACKBOARD_X, -30, LEFT),
            backboardCenter = new EditablePose(BACKBOARD_X, -33, LEFT),
            backboardRight = new EditablePose(BACKBOARD_X, -41, LEFT),
            parkingLeft = new EditablePose(48.5, -10, LEFT),
            parkingRight = new EditablePose(48.5, -56, LEFT),
            spikeDodgeStageDoor = new EditablePose(28, -9, LEFT),
            stageDoor = new EditablePose(13, -11.5, LEFT),
            innerTruss = new EditablePose(-8, -34.5, LEFT),
            outerTruss = new EditablePose(23.5, -58, LEFT),
            outerTruss2 = new EditablePose(-23.5, -58, LEFT),
            pixelStack1 = new EditablePose(-57.5, -12.5, LEFT),
            pixelStack3 = new EditablePose(-57.5, -35, LEFT),
            scorePrepStageDoor = new EditablePose(BACKBOARD_X - 4, -11.5, LEFT),
            scorePrepTruss = new EditablePose(BACKBOARD_X - 4, -58, LEFT),
            frontBackboardLeft = new EditablePose(BACKBOARD_X - 4, backboardLeft.y, LEFT),
            frontBackboardCenter = new EditablePose(BACKBOARD_X - 4, backboardCenter.y, LEFT),
            frontBackboardRight = new EditablePose(BACKBOARD_X - 4, backboardRight.y, LEFT);

    private static EditablePose mainSpike, pixelStack, whiteScoring, yellowScoring, transition, frontWhiteScoring, prep;

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
                            transition = isUnderTruss ? outerTruss : spikeDodgeStageDoor;
                            frontWhiteScoring = isRed ? (isUnderTruss ? frontBackboardRight : frontBackboardCenter) : (isUnderTruss ? frontBackboardCenter : frontBackboardLeft);
                            whiteScoring = isUnderTruss ? backboardRight : backboardLeft;
                            break;
                        case 1:
                            mainSpike = isRed ? spikeCenterRed : spikeCenterBlue;
                            yellowScoring = backboardCenter;
                            transition = isUnderTruss ? outerTruss : spikeDodgeStageDoor;
                            frontWhiteScoring = isUnderTruss ? frontBackboardRight : frontBackboardLeft;
                            whiteScoring = isUnderTruss ? backboardRight : backboardLeft;
                            break;
                        case 2:
                            mainSpike = isRed ? spikeRightRed : spikeLeftBlue;
                            yellowScoring = isRed ? backboardRight : backboardLeft;
                            transition = isUnderTruss ? outerTruss : spikeDodgeStageDoor;
                            frontWhiteScoring = isRed ? (isUnderTruss ? frontBackboardCenter : frontBackboardLeft) : (isUnderTruss ? frontBackboardRight : frontBackboardCenter);
                            whiteScoring = isUnderTruss ? backboardRight : backboardLeft;
                            break;
                    }

                    prep = isUnderTruss ? scorePrepTruss : scorePrepStageDoor;
                    pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;

                    Pose2d startPose = start.byAlliancePose2d();
                    drive.setPoseEstimate(startPose);
                    TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(startPose);

                    scorePurplePixel(builder, randomization);
                    scoreYellowPixel(builder);

                    //retractSlides(builder);
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

        //builder.addTemporalMarker(() -> robot.purplePixel.setActivated(true));
    }

    private static void scoreYellowPixel(TrajectorySequenceBuilder builder) {
        builder.lineToSplineHeading(yellowScoring.byAlliancePose2d());
        //score(builder, false);
    }

    private static void getWhitePixels(TrajectorySequenceBuilder builder, int cycle) {
        builder.lineTo(transition.byAllianceVec());

        if (isUnderTruss) builder.lineTo(outerTruss2.byAllianceVec());

        builder.lineTo(pixelStack.byAllianceVec());
    }

    private static void scoreWhitePixels(TrajectorySequenceBuilder builder) {

        if (isUnderTruss) builder.lineTo(outerTruss2.byAllianceVec());

        builder.lineTo(prep.byAllianceVec())
                .lineTo(frontWhiteScoring.byAllianceVec())
                .lineTo(whiteScoring.byAllianceVec());

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
            EditablePose pose = new EditablePose(x, y, heading);
            if (!isRed) {
                pose.y *= -1;
                pose.heading *= -1;
            }
            return pose;
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
    }
}