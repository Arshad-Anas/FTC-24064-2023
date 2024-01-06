package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    static boolean isRed = true, isRight = true, isParkedLeft = false;

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static EditablePose
            startPoseBlue = new EditablePose(X_START_RIGHT, -100, BACKWARD),
            startPoseRed = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            centerSpikeBlue = new EditablePose((X_START_RIGHT + 3.5), 33.5, BACKWARD),
            centerSpikeRed = new EditablePose((X_START_RIGHT + 3.5), -33.5, FORWARD),
            leftSpike = new EditablePose(7, -40, toRadians(120)),
            rightSpike = new EditablePose(24 - leftSpike.x, leftSpike.y, LEFT - leftSpike.heading),
            blueBackboard = new EditablePose(48, 34, LEFT),
            redBackboard = new EditablePose(48, -34, LEFT),
            blueParkingLeft = new EditablePose(52, -14, toRadians(165)),
            blueParkingRight = new EditablePose(51, -54, toRadians(200));

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double side = isRed ? 1 : -1;

        double
                OUTTAKE_WAIT_TIME = 0.25,
                INTAKE_WAIT_TIME = 0.65,
                SCORING_WAIT_TIME = 0.75,
                OPEN_FLAP_WAIT_TIME = 0.25;

        Pose2d startPoseBlue = MeepMeepTesting.startPoseBlue.byBoth().toPose2d();
        Pose2d startPoseRed = MeepMeepTesting.startPoseRed.byBoth().toPose2d();
        Pose2d centerSpikeBlue = MeepMeepTesting.centerSpikeBlue.byBoth().toPose2d();
        Pose2d centerSpikeRed = MeepMeepTesting.centerSpikeRed.byBoth().toPose2d();
        Pose2d blueBackboard = MeepMeepTesting.blueBackboard.byBoth().toPose2d();
        Pose2d redBackboard = MeepMeepTesting.redBackboard.byBoth().toPose2d();
        Pose2d blueParkingLeft = MeepMeepTesting.blueParkingLeft.byBoth().toPose2d();
        Pose2d blueParkingRight = MeepMeepTesting.blueParkingRight.byBoth().toPose2d();

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(30, 30, toRadians(60), toRadians(60), 16.02362205)
                .setDimensions(16.2981681102, 17.0079)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(isRed ? startPoseRed : startPoseBlue)
                                .splineTo(isRed ? centerSpikeRed.vec() : centerSpikeBlue.vec(), FORWARD)
                                /*
                                   Starts outtake 0.5 seconds after prev. action, then waits 0.25 seconds before stopping the outtake
                                   Then stops after 1 second
                                 */
                                // .UNSTABLE_addTemporalMarker(0.5, () -> intake.set(0.35))
                                // .addTemporalMarker(0.5 + OUTTAKE_WAIT_TIME, () -> intake.set(0))
                                .lineToLinearHeading(isRed ? redBackboard : blueBackboard)
                                /*
                                 Starts the lift by updating target to row 0, then executes commands to do so (within timing)
                                 It will activate flap to open, releasing the pixels
                                 After doing that, it'll retract back to target row -1
                                */
                                // .UNSTABLE_addTemporalMarker(0.5, () -> {
                                //      robot.lift.setTargetRow(0);
                                //      robot.lift.updateTarget();
                                // })
                                // .addTemporalMarker(0.5 + OPEN_FLAP_WAIT_TIME, () -> robot.arm.setFlap(true))
                                // .addTemporalMarker((0.5 + OPEN_FLAP_WAIT_TIME) + SCORING_WAIT_TIME, () -> robot.arm.setArm(true))
                                .lineToSplineHeading(isParkedLeft ? blueParkingLeft : blueParkingRight)
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