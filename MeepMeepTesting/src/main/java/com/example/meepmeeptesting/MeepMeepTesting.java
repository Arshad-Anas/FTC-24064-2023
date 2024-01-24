package com.example.meepmeeptesting;

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
            X_START_RIGHT = 12;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static int propPlacement = 2;

    public static EditablePose
            botStartRed = new EditablePose(X_START_BOTTOM, -61.788975, FORWARD),
            botLeftSpikeRed = new EditablePose(-47, -44, FORWARD),
            botCenterSpikeRed = new EditablePose(-41, -38, FORWARD),
            botRightSpikeRed = new EditablePose(-32, -39, toRadians(145 - 90)),
            botLeftPixelDodgeRed = new EditablePose(-57, -44, LEFT),
            botCenterRightPixelDodgeRed = new EditablePose (-57, -38, LEFT),
            botWhitePixelRed = new EditablePose(-57,-24, LEFT),
            botStageDoorRed = new EditablePose(-25, -10, RIGHT),
            botTransitionRed = new EditablePose(25, -9, RIGHT),
            botCenterBackdropRed = new EditablePose(48, -35, LEFT),
            botLeftBackdropRed = new EditablePose(48, -29, LEFT),
            botRightBackdropRed = new EditablePose(48, -41, LEFT);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);
        EditablePose prop, dodge, yellowPixel, botWhitePixelScoring;
        prop = dodge = yellowPixel = botWhitePixelScoring = null;

        switch (propPlacement) {
            case 0:
                prop = isRed ? botLeftSpikeRed : botRightSpikeRed;
                dodge = isRed ? botLeftPixelDodgeRed : botCenterRightPixelDodgeRed;
                yellowPixel = isRed ? botLeftBackdropRed : botRightBackdropRed;
                botWhitePixelScoring = isRed ? botCenterBackdropRed : botLeftBackdropRed;
                break;
            case 1:
                prop = botCenterSpikeRed;
                dodge = botCenterRightPixelDodgeRed;
                yellowPixel = botCenterBackdropRed;
                botWhitePixelScoring = botLeftBackdropRed;
                break;
            case 2:
                prop = isRed ? botRightSpikeRed : botLeftSpikeRed;
                dodge = isRed ? botCenterRightPixelDodgeRed : botLeftPixelDodgeRed;
                yellowPixel = isRed ? botRightBackdropRed : botLeftBackdropRed;
                botWhitePixelScoring = isRed ? botLeftBackdropRed : botCenterBackdropRed;
                break;
        }

        Pose2d start = botStartRed.byAlliancePose2d();
        //robot.drivetrain.setPoseEstimate(start);

        EditablePose finalProp = prop;
        EditablePose finalDodge = dodge;
        EditablePose finalBotWhitePixelScoring = botWhitePixelScoring;
        EditablePose finalYellowPixel = yellowPixel;
        EditablePose botRightSpikePrepRed = new EditablePose(-38.5, -34, FORWARD);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.2765, 60.3457, toRadians(174.5386897712936), toRadians(60), 17.38)
                .setDimensions(16.2981681102, 17.0079)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start)
                                .lineTo(botRightSpikePrepRed.byAllianceVec())
                                .lineToSplineHeading(finalProp.byAlliancePose2d())
                                .back(8)
                                .lineToSplineHeading(finalDodge.byAlliancePose2d())
                                .lineToLinearHeading(botWhitePixelRed.byAlliancePose2d())
                                .forward(5)
                                .turn(toRadians(isRed ? -35 : 35))
                                .turn(toRadians(isRed ? 15 : -15))
                                //.addTemporalMarker(() -> robot.intake.set(-1)) // Intake
                                .forward(0.3)
                                .waitSeconds(2)
                                .back(5.3)
                                //.addTemporalMarker(() -> robot.intake.set(1)) // Outtake
                                .strafeRight(isRed ? 4 : -4)
                                //.addTemporalMarker(() -> robot.intake.set(0)) // Stop outtaking
                                .splineToSplineHeading(botStageDoorRed.byAlliancePose2d(), Math.toRadians(0))
                                .splineTo(botTransitionRed.byAllianceVec(), Math.toRadians(0))
                                //.addTemporalMarker(() -> robot.lift.setToAutonHeight()) // Lift and arm extend
                                .lineToSplineHeading(finalBotWhitePixelScoring.byAlliancePose2d())
                                //.addTemporalMarker(() -> robot.arm.setFlap(false)) // Open flap
                                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.arm.setFlap(true)) // Close flap 0.1 seconds after opening them
                                .lineToSplineHeading(finalYellowPixel.byAlliancePose2d())
                                //.addTemporalMarker(() -> robot.arm.setFlap(false))
                                //.UNSTABLE_addTemporalMarkerOffset(1, () -> robot.arm.toggleArm());
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