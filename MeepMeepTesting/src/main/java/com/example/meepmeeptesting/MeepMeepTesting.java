package com.example.meepmeeptesting;

import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;


public class MeepMeepTesting {

    static boolean isRed = true, isParkedMiddle = false;

    static final boolean isTop = true;

    public static double
            X_START_BOTTOM = -37,
            X_START_RIGHT = 12;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static int propPlacement = 1;

    public static double
            BACKBOARD_X = 58;

    // Bottom
    public static EditablePose
            botStartRed = new EditablePose(X_START_BOTTOM, -61.788975, FORWARD),
            botLeftSpikeRed = new EditablePose(-50, -39, FORWARD),
            botCenterSpikeRed = new EditablePose(-41, -32, FORWARD),
            botRightSpikeRed = new EditablePose(-37, -35.5, FORWARD),
            botLeftPixelDodgeRed = new EditablePose(-36.5, -50, FORWARD),
            botCenterPixelDodgeRed = new EditablePose (-53, -38, FORWARD),
            botCenterPixelDodgeRed2 = new EditablePose(-53, -10, FORWARD),
            botStageDoorRed = new EditablePose(-36.5, -7, RIGHT),
            botTransitionRed = new EditablePose(40, -7, LEFT),
            botLeftBackdropRed = new EditablePose(BACKBOARD_X - 2, -27, LEFT),
            botCenterBackdropRed = new EditablePose(BACKBOARD_X, -31, LEFT),
            botRightBackdropRed = new EditablePose(BACKBOARD_X - 2.75, -52, LEFT),

            // Top
            topLeftBackdropRed = new EditablePose(BACKBOARD_X - 7, -28.5, LEFT),
            topCenterBackdropRed = new EditablePose(BACKBOARD_X - 7, -35, LEFT),
            topRightBackdropRed = new EditablePose(BACKBOARD_X - 7, -41.5, LEFT),
            topParkingRed = new EditablePose(47.5, -60, LEFT);

    private static EditablePose start, prop, dodge, yellowPixel;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        switch (propPlacement) {
            case 0:
                prop = isTop ? (isRed ? botRightSpikeRed : botLeftSpikeRed) : (isRed ? botLeftSpikeRed : botRightSpikeRed);
                dodge = botLeftPixelDodgeRed;
                yellowPixel = isTop ? (isRed ? topLeftBackdropRed : topRightBackdropRed) : (isRed ? botLeftBackdropRed : botRightBackdropRed);
                break;
            case 1:
                prop = botCenterSpikeRed;
                dodge = botCenterPixelDodgeRed;
                yellowPixel = isTop? topCenterBackdropRed : botCenterBackdropRed;
                break;
            case 2:
                prop = isTop ? (isRed ? botLeftSpikeRed : botRightSpikeRed) : (isRed ? botRightSpikeRed : botLeftSpikeRed);
                dodge = botLeftPixelDodgeRed;
                yellowPixel = isTop ? (isRed ? topRightBackdropRed : topLeftBackdropRed) : (isRed ? botRightBackdropRed : botLeftBackdropRed);
                break;
        }

        start = botStartRed;
        //robot.drivetrain.setPoseEstimate(start);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.2765, 60.3457, toRadians(174.5386897712936), toRadians(60), 17.38)
                .setDimensions(16.2981681102, 17.0079)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(start.bySide().byAlliancePose2d())
                                .lineToSplineHeading(prop.bySide().byAlliancePose2d())
//                                .turn(toRadians(isRed ? 90 : -90)) // left
//                                .forward(3) // left
                                .back(11) // If not left
                                .lineToSplineHeading(yellowPixel.byAlliancePose2d())
                                .forward(3)
                                .lineToSplineHeading(topParkingRed.byAlliancePose2d())
                                .back(13)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.85f)
                .addEntity(myBot)
                .start();
    }

    private void addPurplePixel(TrajectorySequenceBuilder builder) {
        builder.lineToSplineHeading(prop.bySide().byAlliancePose2d());

        if (!isTop) {
            if (isLeft() || isCenter()) {
                builder.back(11)
                        .lineToSplineHeading(dodge.byAlliancePose2d());
            }

            if (isCenter())
                builder.lineToSplineHeading(botCenterPixelDodgeRed2.byAlliancePose2d());

            if (isRight()) {
                builder.turn(toRadians(isRed ? -90 : 90))
                        .turn(toRadians(isRed ? 90 : -90));
            }
        }

//        builder.addTemporalMarker(() -> robot.intake.set(0.25))
//                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.intake.set(0));
    }

    private boolean isCenter() {
        return propPlacement == 1;
    }

    private boolean isLeft() {
        return isRed && propPlacement == 0 || !isRed && propPlacement == 2;
    }

    private boolean isRight() {
        return isRed && propPlacement == 2 || !isRed && propPlacement == 0;
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

        private EditablePose bySide() {
            if (isTop) {
                x = x * -1 - 23.5;
            }
            return this;
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