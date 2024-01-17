package com.example.meepmeeptesting;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class MeepMeepTesting {

    static boolean isRed = true, isParkedLeft = false;

    static final boolean isRight = true;

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static int spikeNum = 2;

    public static EditablePose
            topStart = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            topCenterSpike = new EditablePose((X_START_RIGHT + 3.5), -33.5, FORWARD),
            topLeftSpike = new EditablePose(7, -41, toRadians(120)),
            topRightSpike = new EditablePose(24 - topLeftSpike.x, topLeftSpike.y, LEFT - topLeftSpike.heading),
            topBackboard = new EditablePose(48, -34, RIGHT),
            topParkingLeft = new EditablePose(52, -14, toRadians(165)),
            topParkingRight = new EditablePose(51, -56, toRadians(200));

    public static Pose2d mainSpike = null;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double
                OUTTAKE_WAIT_TIME = 0.25,
                INTAKE_WAIT_TIME = 0.65,
                SCORING_WAIT_TIME = 0.75,
                OPEN_FLAP_WAIT_TIME = 0.25;

        Pose2d startPose = MeepMeepTesting.topStart.byAlliance().toPose2d();
        Pose2d centerSpike = MeepMeepTesting.topCenterSpike.byAlliance().toPose2d();
        Pose2d leftSpike = MeepMeepTesting.topLeftSpike.byAlliance().toPose2d();
        Pose2d rightSpike = MeepMeepTesting.topRightSpike.byAlliance().toPose2d();
        Pose2d backboard = MeepMeepTesting.topBackboard.byAlliance().toPose2d();
        Pose2d parkingLeft = MeepMeepTesting.topParkingLeft.byAlliance().toPose2d();
        Pose2d parkingRight = MeepMeepTesting.topParkingRight.byAlliance().toPose2d();

        switch (spikeNum) {
            case 0: {
                mainSpike = leftSpike;
                break;
            }

            case 1: {
                mainSpike = centerSpike;
                break;
            }

            case 2: {
                mainSpike = rightSpike;
                break;
            }
        }

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50.2765, 60.3457, toRadians(174.5386897712936), toRadians(60), 17.38)
                .setDimensions(16.2981681102, 17.0079)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)
                                .splineTo(mainSpike.vec(), mainSpike.getHeading())
                                /*
                                   Starts outtake 0.5 seconds after prev. action, then waits 0.25 seconds before stopping the outtake
                                   Then stops after 1 second
                                 */
                                // .UNSTABLE_addTemporalMarkerOffset(0.5, () -> intake.set(0.35))
                                // .addTemporalMarker(0.5 + OUTTAKE_WAIT_TIME, () -> intake.set(0))
                                .strafeRight(spikeNum == 2 ? (isRed ? 8 : -8) : 0.0001)
                                .turn(spikeNum == 2 ? (isRed ? (RIGHT - toRadians(35)) : (RIGHT + toRadians(35))) : 0)
                                .lineToSplineHeading(backboard)
                                /*
                                    Do april tag stuff here because now we can scan
                                 */
                                .turn(LEFT)
                                /*
                                 Starts the lift by updating target to row 0, then executes commands to do so (within timing)
                                 It will activate flap to open, releasing the pixels
                                 After doing that, it'll retract back to target row -1
                                */
                                // .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                                //      robot.lift.setTargetRow(0);
                                //      robot.lift.updateTarget();
                                // })
                                // .addTemporalMarker(0.5 + OPEN_FLAP_WAIT_TIME, () -> robot.arm.setFlap(true))
                                // .addTemporalMarker((0.5 + OPEN_FLAP_WAIT_TIME) + SCORING_WAIT_TIME, () -> robot.arm.setArm(true))
                                .lineToSplineHeading((isParkedLeft ? parkingLeft : parkingRight))
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
            if (!isRed) y *= -1;
            if (!isRed) heading *= -1;
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