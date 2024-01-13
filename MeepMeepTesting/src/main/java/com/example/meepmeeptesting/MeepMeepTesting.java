package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    static boolean isRed = false, isRight = false, isParkedLeft = false;

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static EditablePose
         // this is red alliance
            startPoseRed = new EditablePose(X_START_LEFT, -61.788975, FORWARD),
            centerSpikeRed = new EditablePose(-39, -38,Math.toRadians(90)),
            leftSpikeRed = new EditablePose(-46.5, -47, toRadians(90)),
            rightSpikeRed = new EditablePose(15 + leftSpikeRed.x, 5 + leftSpikeRed.y, Math.toRadians(0)),

          // the below 6 are for center spike red alliance
          //  firstSpikeRed = new EditablePose(-49,-24,Math.toRadians(0)),
            whitePixelRed = new EditablePose(-58,-24,Math.toRadians(-180)),
            stageDoorRed = new EditablePose(-25, -10,Math.toRadians(0)), //15 could also work here if dBRed is removed, but this require further testing
            dBRed = new EditablePose(25,-9,Math.toRadians(0)),
            backDropRed = new EditablePose(49,-35,Math.toRadians(-180)),

            parkingLeftRed = new EditablePose(50, -10, Math.toRadians(-180)),
            parkingRightRed = new EditablePose(50,-60, Math.toRadians(-180)),
            leftSpikemovementRed = new EditablePose(-55,-45, Math.toRadians(180)),

    // This is for blue alliance
            startPoseBlue = new EditablePose(startPoseRed.byAlliance().toPose2d().vec().getX(), 61.788975,BACKWARD),
            centerSpikeBlue = new EditablePose(centerSpikeRed.byAlliance().toPose2d().vec().getX(), 38, Math.toRadians(-90)),
            leftSpikeBlue = new EditablePose(leftSpikeRed.byAlliance().toPose2d().vec().getX(), 47, toRadians(-90)),
            rightSpikeBlue = new EditablePose(rightSpikeRed.byAlliance().toPose2d().vec().getX(), -leftSpikeRed.y, LEFT + leftSpikeRed.heading),

            // the below 6 are for center spike blue alliance
         //   firstSpikeBlue = new EditablePose(firstSpikeRed.byAlliance().toPose2d().vec().getX(),24,Math.toRadians(0)),
            whitePixelBlue = new EditablePose(whitePixelRed.byAlliance().toPose2d().vec().getX(),24,Math.toRadians(180)),
            stageDoorBlue = new EditablePose(stageDoorRed.byAlliance().toPose2d().vec().getX(), 8,Math.toRadians(0)),
            dBBlue = new EditablePose(dBRed.byAlliance().toPose2d().vec().getX(),9,Math.toRadians(0)),
            backDropBlue = new EditablePose(backDropRed.byAlliance().toPose2d().vec().getX(),35,Math.toRadians(180)),
            parkingRightBlue = new EditablePose(parkingRightRed.byAlliance().toPose2d().vec().getX(), 10, Math.toRadians(-180)),
            parkingLeftBlue = new EditablePose(parkingLeftRed.byAlliance().toPose2d().vec().getX(), 60, Math.toRadians(-180)),
            leftSpikemovementBlue = new EditablePose(leftSpikemovementRed.byAlliance().toPose2d().vec().getX(), 45, Math.toRadians(-180));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(770);
        Pose2d startPoseRed = MeepMeepTesting.startPoseRed.byAlliance().toPose2d();
        Pose2d centerSpikeRed = MeepMeepTesting.centerSpikeRed.byAlliance().toPose2d();
        Pose2d leftSpikeRed = MeepMeepTesting.leftSpikeRed.byAlliance().toPose2d();
        Pose2d rightSpikeRed = MeepMeepTesting.rightSpikeRed.byAlliance().toPose2d();
        Pose2d whitePixelRed = MeepMeepTesting.whitePixelRed.byAlliance().toPose2d();
        Pose2d stageDoorRed = MeepMeepTesting.stageDoorRed.byAlliance().toPose2d();
        Pose2d dBRed = MeepMeepTesting.dBRed.byAlliance().toPose2d();
        Pose2d backDropRed = MeepMeepTesting.backDropRed.byAlliance().toPose2d();
        Pose2d parkingRightRed = MeepMeepTesting.parkingRightRed.byAlliance().toPose2d();
        Pose2d parkingLeftRed = MeepMeepTesting.parkingLeftRed.byAlliance().toPose2d();
        Pose2d leftSpikemovementRed = MeepMeepTesting.leftSpikemovementRed.byAlliance().toPose2d();

        Pose2d startPoseBlue = MeepMeepTesting.startPoseBlue.bySide().toPose2d();
        Pose2d centerSpikeBlue = MeepMeepTesting.centerSpikeBlue.toPose2d();
        Pose2d leftSpikeBlue = MeepMeepTesting.leftSpikeBlue.toPose2d();
        Pose2d rightSpikeBlue = MeepMeepTesting.rightSpikeBlue.toPose2d();
        Pose2d whitePixelBlue = MeepMeepTesting.whitePixelBlue.toPose2d();
        Pose2d stageDoorBlue = MeepMeepTesting.stageDoorBlue.toPose2d();
        Pose2d dBBlue = MeepMeepTesting.dBBlue.toPose2d();
        Pose2d backDropBlue = MeepMeepTesting.backDropBlue.toPose2d();
        Pose2d parkingRightBlue = MeepMeepTesting.parkingRightBlue.toPose2d();
        Pose2d parkingLeftBlue = MeepMeepTesting.parkingLeftBlue.toPose2d();
        Pose2d leftSpikemovementBlue = MeepMeepTesting.leftSpikemovementBlue.toPose2d();



        double side = isRed ? 1 : -1;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, toRadians(60), toRadians(60), 16.02362205)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(isRed ? startPoseRed : startPoseBlue)
                             //   .lineToSplineHeading(new Pose2d(-34,-37, Math.toRadians(90)))
                                .lineToSplineHeading(isRed ? leftSpikeRed : leftSpikeBlue)
                        // .splineTo(isRed ? leftSpikeRed.vec() : leftSpikeBlue.vec(), Math.toRadians(0))
                            //    .addDisplacementMarker((){})
                               .lineToSplineHeading(isRed ? leftSpikemovementRed : leftSpikemovementBlue) //DO TURNARY IF LEFT SPIKE
                             //   .lineTo(isRed ? whitePixelRed.vec() : whitePixelBlue.vec())
                                .lineToLinearHeading(isRed ? whitePixelRed : whitePixelBlue)
                                .strafeRight(isRed ? 4 : -4)
                                .splineToSplineHeading(isRed ? stageDoorRed : stageDoorBlue, Math.toRadians(0))
                                .splineTo(isRed ? dBRed.vec() : dBBlue.vec(), Math.toRadians(0))
                                .lineToSplineHeading(isRed ? backDropRed : backDropBlue )
                                .lineTo(isRed ? (isParkedLeft ? parkingLeftRed.vec() : parkingRightRed.vec()) : (isParkedLeft ? parkingLeftBlue.vec() : parkingRightBlue.vec()))
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