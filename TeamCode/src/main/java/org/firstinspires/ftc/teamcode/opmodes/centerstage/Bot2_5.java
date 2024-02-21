package org.firstinspires.ftc.teamcode.opmodes.centerstage;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.BACKWARD;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.LEFT;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.RIGHT;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.aprilTag;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.autonEndPose;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.isRed;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.propSensor;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.robot;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.vision.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.vision.PropSensor;

@Config
@Autonomous(name = "Bottom 2+5", group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class Bot2_5 extends LinearOpMode {
    static boolean
            isParkedMiddle = true,
            isUnderTruss = false,
            doAprilTag = false;

    public static double
            X_START_BOTTOM = -37;

    public static double
            BACKBOARD_X = 51.6,
            ANGLE_1 = 52,
            ANGLE_2 = 46,
            ANGLE_3 = 32.5,
            ANGLE_4 = 20,
            ANGLE_5 = 2;

    public static EditablePose
            botStartRed = new EditablePose(X_START_BOTTOM, -61.788975, BACKWARD),
            botLeftSpikeRed = new EditablePose(-49 , -16, LEFT),
            botLeftSpikeRed2 = new EditablePose(-56,-36, toRadians(210)),
            botCenterSpikeRed = new EditablePose(-50, -22, LEFT),
            botCenterSpikeBlue = new EditablePose(-50 , -25, LEFT),
            botRightSpikeRed = new EditablePose(-33, -35, toRadians(210)),
            botRightSpikeBlue = new EditablePose(-33,-36, toRadians(170)),
            botCenterBackdropRed = new EditablePose(BACKBOARD_X, -34.5, LEFT),
            botLeftBackdropRed = new EditablePose(BACKBOARD_X, -30.5, LEFT),
            botRightBackdropRed = new EditablePose(BACKBOARD_X, -41, LEFT),
            botAudienceSpikeTransitionRed = new EditablePose(-34,-18,toRadians(110)),
            botStageDoor = new EditablePose(25,-10,LEFT),
            botTrussInner = new EditablePose(20,-36,LEFT),
            botTrussOuter = new EditablePose(20,-58,LEFT),
            firstWhitePixelStackRed = new EditablePose(-56.6,-12, LEFT),
            trussTransition = new EditablePose(-53,-58,LEFT),
            thirdWhitePixelStackRed = new EditablePose(-56.6, -35,LEFT);

    private EditablePose mainSpike, pixelStack, whiteScoring, yellowScoring, transition;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize gamepad (ONLY FOR INIT, DON'T CALL DURING WHILE LOOP)
        gamepadEx1 = new GamepadEx(gamepad1);

        robot = new Robot(hardwareMap);

        // Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (keyPressed(1, B)) isRed = true;
            if (keyPressed(1, X)) isRed = false;
            if (keyPressed(1, A)) isParkedMiddle = true;
            if (keyPressed(1, Y)) isParkedMiddle = false;
            if (keyPressed(1, DPAD_UP)) isUnderTruss = true;
            if (keyPressed(1, DPAD_DOWN)) isUnderTruss = false;
            if (keyPressed(1, DPAD_RIGHT)) doAprilTag = false;
            if (keyPressed(1, DPAD_LEFT)) doAprilTag = true;
            mTelemetry.addLine("B - RED | X - BLUE |");
            mTelemetry.addLine("A - PARK MIDDLE | Y - PARK CORNER");
            mTelemetry.addLine("D-pad up - UNDER TRUSS | D-pad down - UNDER DOOR");
            mTelemetry.addLine("D-pad right - NO APRIL TAG | D-pad left - APRIL TAG");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + " " + (isParkedMiddle ? "PARK MIDDLE" : "PARK CORNER") + " " + (isUnderTruss ? "UNDER TRUSS" : "UNDER DOOR") + " " + (doAprilTag ? "APRIL TAG" : "NO APRIL TAG"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }

        TrajectorySequence[] trajectories = {getTrajectory(0), getTrajectory(1), getTrajectory(2)};
        TrajectorySequence trajectory = null;

        propSensor = new PropSensor(hardwareMap, isRed);

        while (!propSensor.getIsOpened()) {
            mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isParkedMiddle ? "PARK MIDDLE" : "PARK CORNER") + " " + (isUnderTruss ? "UNDER TRUSS" : "UNDER DOOR") + " " + (doAprilTag ? "APRIL TAG" : "NO APRIL TAG"));
            mTelemetry.addLine("Camera is not open");
            mTelemetry.update();
        }

        while (!isStarted() && !isStopRequested()) {
            int randomization = propSensor.propPosition();
            trajectory = trajectories[randomization];
            mTelemetry.addData("Predicted Prop Placement", randomization);
            mTelemetry.update();
            sleep(50);
        }

//        propSensor.getCamera().stopStreaming();
        propSensor.getCamera().closeCameraDeviceAsync(() -> {
            mTelemetry.addLine("Camera closed");
            mTelemetry.update();
        });

        aprilTag = new AprilTagLocalization(hardwareMap);
        Pose2d aprilTagPose = null;

        robot.drivetrain.followTrajectorySequenceAsync(trajectory);

        while (opModeIsActive()) {
            if (!opModeIsActive()) {
                return;
            }

            if (doAprilTag) {
                aprilTagPose = aprilTag.getPoseEstimate();
                if (aprilTagPose != null) {
                    robot.drivetrain.getLocalizer().setPoseEstimate(aprilTagPose);
                }
            }

            robot.readSensors();

            if (doAprilTag) {
                mTelemetry.addData("April Tag pose", aprilTagPose);
                mTelemetry.update();
            }

            robot.drivetrain.update();
            robot.run();
        }

        autonEndPose = robot.drivetrain.getPoseEstimate();
    }

    private TrajectorySequence getTrajectory(int randomization) {
        switch (randomization) {
            case 0:
                mainSpike = isRed ?  botLeftSpikeRed : botRightSpikeBlue;
                yellowScoring = isRed ? botLeftBackdropRed : botRightBackdropRed;
                transition = isUnderTruss ? botTrussOuter : botStageDoor;
                pixelStack = isUnderTruss ? thirdWhitePixelStackRed : firstWhitePixelStackRed;
                whiteScoring = isUnderTruss ? botRightBackdropRed : botLeftBackdropRed;
                break;
            case 1:
                mainSpike = isRed ? botCenterSpikeRed: botCenterSpikeBlue;
                yellowScoring = botCenterBackdropRed;
                transition = isUnderTruss ? botTrussInner : botStageDoor;
                pixelStack = isUnderTruss ? thirdWhitePixelStackRed : firstWhitePixelStackRed;
                whiteScoring = isUnderTruss ? botCenterBackdropRed : botLeftBackdropRed;
                break;
            case 2:
                mainSpike = isRed ? botRightSpikeRed : botLeftSpikeRed;
                yellowScoring = isRed ? botRightBackdropRed : botLeftBackdropRed;
                transition = isUnderTruss ? botTrussOuter : botStageDoor;
                pixelStack = isUnderTruss ? thirdWhitePixelStackRed : firstWhitePixelStackRed;
                whiteScoring = isUnderTruss ? botRightBackdropRed : botLeftBackdropRed;
                break;
        }

        Pose2d startPose = botStartRed.byAlliancePose2d();
        robot.drivetrain.setPoseEstimate(startPose);
        TrajectorySequenceBuilder builder = robot.drivetrain.trajectorySequenceBuilder(startPose);

        scorePurplePixel(builder, randomization);
        getFirstWhitePixel(builder, randomization);
        scoreYellowPixel(builder);
        getWhitePixels(builder, randomization ,1);
        scoreWhitePixels(builder, randomization);
        getWhitePixels(builder, randomization, 2);
        scoreWhitePixels(builder, randomization);


        return builder.build();
    }

    private void scorePurplePixel(TrajectorySequenceBuilder builder, int randomization) {
        builder.setTangent(isRed ? BACKWARD : FORWARD);
        if (isAudienceSide(randomization) && !isUnderTruss) {
            builder
                    .lineToSplineHeading(botAudienceSpikeTransitionRed.byAlliancePose2d())
                    .setTangent(botAudienceSpikeTransitionRed.heading)
                    .lineTo(mainSpike.byAllianceVec())
                    .setTangent(LEFT);
        } else if (isAudienceSide(randomization) && isUnderTruss) {
            builder.strafeRight(6)
                    .lineToSplineHeading(botLeftSpikeRed2.byAlliancePose2d());
        } else if (isCenter(randomization) || isBackboardSide(randomization)) {
            builder.lineToSplineHeading(mainSpike.byAlliancePose2d());
        }
    }

    private void getFirstWhitePixel(TrajectorySequenceBuilder builder, int randomization) {
        builder.lineToSplineHeading(pixelStack.byAlliancePose2d());

        intakePixels(builder, 0);

        if (isUnderTruss && !isCenter(randomization)) builder.lineToConstantHeading(trussTransition.byAllianceVec());
    }

    private void scoreYellowPixel(TrajectorySequenceBuilder builder) {
        builder.setTangent(RIGHT)
                .splineTo(transition.byAllianceVec(), RIGHT)
                .splineToConstantHeading(yellowScoring.byAllianceVec(), RIGHT);

        //score(builder, true);
    }

    private void getWhitePixels(TrajectorySequenceBuilder builder, int randomization, int cycle) {
        builder.setTangent(LEFT)
                .splineToConstantHeading(transition.byAllianceVec(), LEFT);

        if (isUnderTruss && randomization != 1)
            builder.splineToConstantHeading(trussTransition.byAllianceVec(),LEFT)
                    .lineToConstantHeading(pixelStack.byAllianceVec());
        else builder.splineTo(pixelStack.byAllianceVec(), pixelStack.heading);

        intakePixels(builder, cycle);
    }

    private void scoreWhitePixels(TrajectorySequenceBuilder builder, int randomization) {
        if (isUnderTruss && randomization != 1)
            builder.lineToConstantHeading(trussTransition.byAllianceVec());
        builder.setTangent(RIGHT);

        builder.splineTo(transition.byAllianceVec(), RIGHT)
                .splineToConstantHeading(whiteScoring.byAllianceVec(), RIGHT);

        //score(builder, true);
    }

    private void intakePixels(TrajectorySequenceBuilder builder, int cycle) {
        builder.addTemporalMarker(() -> {
            robot.rollers.setDeployable(cycle == 0 ? ANGLE_1 : cycle == 1 ? ANGLE_2 : ANGLE_4);
            robot.rollers.intake(-1);
        });

        if (cycle != 0)
            builder.waitSeconds(0.8)
                    .addTemporalMarker(() -> robot.rollers.setDeployable(cycle == 1 ? ANGLE_3 : ANGLE_5))
                    .waitSeconds(0.8);

        builder.UNSTABLE_addTemporalMarkerOffset(1, () -> robot.rollers.resetDeployable())
                .UNSTABLE_addTemporalMarkerOffset(2, () -> robot.rollers.intake(0));
    }

    private void score(TrajectorySequenceBuilder builder, boolean isWhite) {
        builder.addTemporalMarker(() -> robot.lift.setToAutonHeight(isWhite ? 300 : 0))
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    robot.arm.setArm(true);
                    robot.wrist.setActivated(true);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> robot.arm.setFlap(false))
                .waitSeconds(0.8)
                .addTemporalMarker(() -> robot.lift.setToAutonHeight(isWhite ? 700 : 400))
                .waitSeconds(0.7)
                .addTemporalMarker(() -> {
                    robot.arm.setArm(false);
                    robot.wrist.setActivated(false);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> robot.lift.retract());
    }

    private boolean isCenter(int randomization) {
        return randomization == 1;
    }

    private boolean isAudienceSide(int randomization) {
        return isRed && randomization == 0 || !isRed && randomization == 2;
    }

    private boolean isBackboardSide(int randomization) {
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
        private Bot2_5.EditablePose byAlliance() {
            Bot2_5.EditablePose pose = new Bot2_5.EditablePose(x, y, heading);
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
