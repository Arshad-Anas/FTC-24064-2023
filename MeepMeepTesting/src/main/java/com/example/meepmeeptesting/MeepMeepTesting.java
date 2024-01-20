package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    static boolean isRed = true, isParkedMiddle = false;

    static final boolean isRight = true;

    public static double
            X_START_BOTTOM = -37,
            X_START_RIGHT = 12,
            TIME_START_INTAKE = 5,
            TIME_START_OUTTAKE = 9,
            TIME_STOP_OUTTAKE = 12,
            TIME_LIFT_SLIDES = 15;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static int propPlacement = 2;

    public static EditablePose
            botStartRed = new EditablePose(X_START_BOTTOM, -61.788975, FORWARD),
            botLeftSpikeRed = new EditablePose(-47, -44, FORWARD),
            botCenterSpikeRed = new EditablePose(-39, -37, FORWARD),
            botRightSpikeRed = new EditablePose(-33, -34, toRadians(130 - 90)),
            botLeftPixelDodgeRed = new EditablePose(-53, -44, LEFT),
            botCenterPixelDodgeRed = new EditablePose (-53, -38, LEFT),
            botRightPixelDodgeRed = new EditablePose(-50, -33, LEFT),
            botWhitePixelRed = new EditablePose(-53,-24, LEFT),
            botStageDoorRed = new EditablePose(-25, -10, RIGHT),
            botTransitionRed = new EditablePose(25, -9, RIGHT),
            botCenterBackdropRed = new EditablePose(48, -35, LEFT),
            botLeftBackdropRed = new EditablePose(48, -29, LEFT),
            botRightBackdropRed = new EditablePose(48, -41, LEFT),
            botParkingLeftRed = new EditablePose(47.5, -10, LEFT),
            botParkingRightRed = new EditablePose(47.5, -60, LEFT);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        EditablePose prop, dodge, yellowPixel, botWhitePixelScoring, parking;
        prop = dodge = yellowPixel = botWhitePixelScoring = null;

        switch (propPlacement) {
            case 0:
                prop = isRed ? botLeftSpikeRed : botRightSpikeRed;
                dodge = isRed ? botLeftPixelDodgeRed : botRightPixelDodgeRed;
                yellowPixel = isRed ? botLeftBackdropRed : botRightBackdropRed;
                botWhitePixelScoring = isRed ? botCenterBackdropRed : botLeftBackdropRed;
                break;
            case 1:
                prop = botCenterSpikeRed;
                dodge = botCenterPixelDodgeRed;
                yellowPixel = botCenterBackdropRed;
                botWhitePixelScoring = botLeftBackdropRed;
                break;
            case 2:
                prop = isRed ? botRightSpikeRed : botLeftSpikeRed;
                dodge = isRed ? botRightPixelDodgeRed : botLeftPixelDodgeRed;
                yellowPixel = isRed ? botRightBackdropRed : botLeftBackdropRed;
                botWhitePixelScoring = isRed ? botLeftBackdropRed : botCenterBackdropRed;
                break;
        }
        parking = isParkedMiddle ? botParkingLeftRed : botParkingRightRed;

        Pose2d start = botStartRed.byAlliancePose2d();
        //robot.drivetrain.setPoseEstimate(start);

        EditablePose finalProp = prop;
        EditablePose finalDodge = dodge;
        EditablePose finalBotWhitePixelScoring = botWhitePixelScoring;
        EditablePose finalYellowPixel = yellowPixel;
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.2765, 60.3457, toRadians(174.5386897712936), toRadians(60), 17.38)
                .setDimensions(16.2981681102, 17.0079)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start)
                                .lineToSplineHeading(finalProp.byAlliancePose2d())
                                .back(8)
                                .lineToSplineHeading(finalDodge.byAlliancePose2d())
                                .lineToLinearHeading(botWhitePixelRed.byAlliancePose2d())
                                .forward(2)
                                //.addTemporalMarker(TIME_START_INTAKE, () -> robot.intake.set(-1))
                                .waitSeconds(3)
                                .back(2)
                                //.addTemporalMarker(TIME_START_OUTTAKE, () -> robot.intake.set(1))
                                //.addTemporalMarker(TIME_STOP_OUTTAKE, () -> robot.intake.set(0))
                                .strafeRight(isRed ? 4 : -4)
                                .splineToSplineHeading(botStageDoorRed.byAlliancePose2d(), Math.toRadians(0))
                                .splineTo(botTransitionRed.byAllianceVec(), Math.toRadians(0))
                                .lineToSplineHeading(finalBotWhitePixelScoring.byAlliancePose2d())
                                //.addTemporalMarker(TIME_LIFT_EXTEND, () -> robot.lift.setToAutonHeight())
                                //.addTemporalMarker(TIME_LIFT_EXTEND + 0.2, () -> robot.arm.setArm(true))
                                //.addTemporalMarker(TIME_LIFT_EXTEND + 0.4, () -> robot.arm.setFlap(false))
                                //.addTemporalMarker(TIME_LIFT_EXTEND + 2, () -> robot.arm.toggle())
                                .lineToSplineHeading(finalYellowPixel.byAlliancePose2d())
                                .lineTo(parking.byAllianceVec())
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