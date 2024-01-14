package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public final class MeepMeepTesting {

    static boolean isRed = false, isRight = false, isParkedLeft = true;
//    static int propPlacement = 1;

    public static double
            X_START_BOTTOM = -35,
            X_START_TOP = 12;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static EditablePose
            // This is red alliance
            botStartPoseRed = new EditablePose(X_START_BOTTOM, -61.788975, FORWARD),
            botCenterSpikeRed = new EditablePose(-39, -38,Math.toRadians(90)),
            botLeftSpikeRed = new EditablePose(-46.5, -47, toRadians(90)),
            botRightSpikeRed = new EditablePose(11 + botLeftSpikeRed.x, 13 + botLeftSpikeRed.y, Math.toRadians(0)),

    // The below 6 are for center spike red alliance
    botWhitePixelRed = new EditablePose(-58,-24,Math.toRadians(-180)),
            botStageDoorRed = new EditablePose(-25, -10,Math.toRadians(0)), //15 could also work here if dBRed is removed, but this require further testing
            botTransitionRed = new EditablePose(25,-9,Math.toRadians(0)),
            botCenterBackdropRed = new EditablePose(49,-35,Math.toRadians(-180)),
            botLeftBackdropRed = new EditablePose(49,-30, Math.toRadians(-180)),
            botRightBackdropRed = new EditablePose(49,-41, Math.toRadians(-180)),
            botParkingLeftRed = new EditablePose(50, -10, Math.toRadians(-180)),
            botParkingRightRed = new EditablePose(50,-60, Math.toRadians(-180)),
            botLeftSpikeMovementRed = new EditablePose(-55,-45, Math.toRadians(180)),

    // This is for blue alliance
    botStartBlue = new EditablePose(botStartPoseRed.byAlliance().x, 61.788975,BACKWARD),
            botCenterSpikeBlue = new EditablePose(botCenterSpikeRed.byAlliance().x, 38, Math.toRadians(-90)),
            botLeftSpikeBlue = new EditablePose(botLeftSpikeRed.byAlliance().x, 47, toRadians(-90)),
            botRightSpikeBlue = new EditablePose(botRightSpikeRed.byAlliance().x, 34, Math.toRadians(0)),

    // The below 6 are for center spike blue alliance
    botWhitePixelBlue = new EditablePose(botWhitePixelRed.byAlliance().x,24,Math.toRadians(180)),
            botStageDoorBlue = new EditablePose(botStageDoorRed.byAlliance().x, 8,Math.toRadians(0)),
            botTransitionBlue = new EditablePose(botTransitionRed.byAlliance().x,9,Math.toRadians(0)),
            botCenterBackdropBlue = new EditablePose(botCenterBackdropRed.byAlliance().x,35,Math.toRadians(180)),
            botLeftBackdropBlue = new EditablePose(botLeftBackdropRed.byAlliance().x, 41, Math.toRadians(180)),
            botRightBackdropBlue = new EditablePose(botRightBackdropRed.byAlliance().x, 29, Math.toRadians(180)),
            botParkingRightBlue = new EditablePose(botParkingRightRed.byAlliance().x, 10, Math.toRadians(-180)),
            botParkingLeftBlue = new EditablePose(botParkingLeftRed.byAlliance().x, 60, Math.toRadians(-180)),
            botLeftSpikeMovementBlue = new EditablePose(botLeftSpikeMovementRed.byAlliance().x, 45, Math.toRadians(-180));

    static Pose2d mainSpikeMark = null;
    static Pose2d yellowPixel = null;
    static Pose2d botWhitePixelScoring = null;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(770);

        Pose2d botStartRed = MeepMeepTesting.botStartPoseRed.byAlliance().toPose2d();
        Pose2d botCenterSpikeRed = MeepMeepTesting.botCenterSpikeRed.byAlliance().toPose2d();
        Pose2d botLeftSpikeRed = MeepMeepTesting.botLeftSpikeRed.byAlliance().toPose2d();
        Pose2d botRightSpikeRed = MeepMeepTesting.botRightSpikeRed.byAlliance().toPose2d();
        Pose2d botWhitePixelRed = MeepMeepTesting.botWhitePixelRed.byAlliance().toPose2d();
        Pose2d botStageDoorRed = MeepMeepTesting.botStageDoorRed.byAlliance().toPose2d();
        Pose2d botTransitionRed = MeepMeepTesting.botTransitionRed.byAlliance().toPose2d();
        Pose2d botCenterBackdropRed = MeepMeepTesting.botCenterBackdropRed.byAlliance().toPose2d();
        Pose2d botLeftBackdropRed = MeepMeepTesting.botLeftBackdropRed.byAlliance().toPose2d();
        Pose2d botRightBackdropRed = MeepMeepTesting.botRightBackdropRed.byAlliance().toPose2d();
        Pose2d botParkingRightRed = MeepMeepTesting.botParkingRightRed.byAlliance().toPose2d();
        Pose2d botParkingLeftRed = MeepMeepTesting.botParkingLeftRed.byAlliance().toPose2d();
        Pose2d botLeftSpikeMovementRed = MeepMeepTesting.botLeftSpikeMovementRed.byAlliance().toPose2d();

        // Blue bottom
        Pose2d botStartBlue = MeepMeepTesting.botStartBlue.bySide().toPose2d();
        Pose2d botCenterSpikeBlue = MeepMeepTesting.botCenterSpikeBlue.toPose2d();
        Pose2d botLeftSpikeBlue = MeepMeepTesting.botLeftSpikeBlue.toPose2d();
        Pose2d botRightSpikeBlue = MeepMeepTesting.botRightSpikeBlue.toPose2d();
        Pose2d botWhitePixelBlue = MeepMeepTesting.botWhitePixelBlue.toPose2d();
        Pose2d botStageDoorBlue = MeepMeepTesting.botStageDoorBlue.toPose2d();
        Pose2d botTransitionBlue = MeepMeepTesting.botTransitionBlue.toPose2d();
        Pose2d botCenterBackdropBlue = MeepMeepTesting.botCenterBackdropBlue.toPose2d();
        Pose2d botLeftBackdropBlue = MeepMeepTesting.botLeftBackdropBlue.toPose2d();
        Pose2d botRightBackdropBlue = MeepMeepTesting.botRightBackdropBlue.toPose2d();
        Pose2d botParkingRightBlue = MeepMeepTesting.botParkingRightBlue.toPose2d();
        Pose2d botParkingLeftBlue = MeepMeepTesting.botParkingLeftBlue.toPose2d();
        Pose2d botLeftSpikeMovementBlue = MeepMeepTesting.botLeftSpikeMovementBlue.toPose2d();

// Global

        Vector2d parking;

        // Local to bottom
        Pose2d botWhitePixels;
        Pose2d botStageDoor;
        Vector2d botTransition;


        int propPlacement = 1;
        switch (propPlacement) {
            case 0: {
                mainSpikeMark = isRed ? botLeftSpikeRed : botLeftSpikeBlue;
                yellowPixel = isRed ? botLeftBackdropRed : botLeftBackdropBlue;
                botWhitePixelScoring = isRed ? botCenterBackdropRed : botRightBackdropBlue;
                break;
            }
            case 1: {
                mainSpikeMark = isRed ? botCenterSpikeRed : botCenterSpikeBlue;
                yellowPixel = isRed ? botCenterBackdropRed : botCenterBackdropBlue;
                botWhitePixelScoring = isRed ? botLeftBackdropRed : botRightBackdropBlue;
                break;
            }
            case 2: {
                mainSpikeMark = isRed ? botRightSpikeRed : botRightSpikeBlue;
                yellowPixel = isRed ? botRightBackdropRed : botRightBackdropBlue;
                botWhitePixelScoring = isRed ? botLeftBackdropRed : botCenterBackdropBlue;
                break;
            }
        }

        if (isRed) {
            botWhitePixels = botWhitePixelRed;
            botStageDoor = botStageDoorRed;
            botTransition = botTransitionRed.vec();
            parking = (isParkedLeft ? botParkingLeftRed : botParkingRightRed).vec();
        } else {
            botWhitePixels = botWhitePixelBlue;
            botStageDoor = botStageDoorBlue;
            botTransition = botTransitionBlue.vec();
            parking = (isParkedLeft ? botParkingLeftBlue : botParkingRightBlue).vec();
        }

//        double side = isRed ? 1 : -1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, toRadians(60), toRadians(60), 16.02362205)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(isRed ? botStartRed : botStartBlue)
                                .lineToSplineHeading(mainSpikeMark)
//                                .lineToSplineHeading(isRed ? botLeftSpikeMovementRed : botLeftSpikeMovementBlue) // TODO DO TERNARY IF LEFT SPIKE
                                .lineToLinearHeading(botWhitePixels)
                                .strafeRight(isRed ? 4 : -4)
                                .splineToSplineHeading(botStageDoor, Math.toRadians(0))
                                .splineTo(botTransition, Math.toRadians(0))
                                .lineToSplineHeading(botWhitePixelScoring)
                                .lineToSplineHeading(yellowPixel)
                                .lineTo(parking)
                                .build());

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
            if (isRight != isRed) x += (X_START_BOTTOM - X_START_TOP);
            return this;
        }

        private EditablePose byBoth() {
            return byAlliance().bySide();
        }

        private EditablePose flipBySide() {
            boolean isRight = MeepMeepTesting.isRight == isRed;
            if (!isRight) heading = PI - heading;
            if (!isRight) x = (X_START_BOTTOM + X_START_TOP) / 2 - x;
            return this;
        }

        private Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }
    }
}