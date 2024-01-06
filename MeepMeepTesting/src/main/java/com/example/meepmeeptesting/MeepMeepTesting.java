package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    static boolean isRed = true, isRight = false;

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static EditablePose
            startPose = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            centerSpike = new EditablePose(X_START_RIGHT, -30, FORWARD),
            leftSpike = new EditablePose(7, -40, toRadians(120)),
            rightSpike = new EditablePose(24 - leftSpike.x, leftSpike.y, LEFT - leftSpike.heading),
            afterSpike = new EditablePose(36, leftSpike.y, LEFT);

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double side = isRed ? 1 : -1;
        Pose2d startPose = MeepMeepTesting.startPose.byBoth().toPose2d();
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, toRadians(60), toRadians(60), 16.02362205)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineTo(new Vector2d(-46,-24), Math.toRadians(0))
                            //    .addDisplacementMarker((){})
                                .strafeLeft(2)
                              //  .lineTo(new Vector2d(0,0), Math.toRadians(-180)
                               // .splineToHeading(new Vector2d(10,-5),Math.toRadians(-180))
                                .splineToSplineHeading(new Pose2d(16, -8), Math.toRadians(0))
                                .splineTo(new Vector2d(17, -8), Math.toRadians(-90))
                                .lineToConstantHeading(new Vector2d(45,-20))
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