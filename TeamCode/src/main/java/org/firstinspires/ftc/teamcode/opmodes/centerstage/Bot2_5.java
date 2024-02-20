package org.firstinspires.ftc.teamcode.opmodes.centerstage;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.BACKWARD;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.LEFT;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.RIGHT;
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
import org.firstinspires.ftc.teamcode.subsystems.centerstage.auton.EditablePose;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.vision.PropSensor;

@Config
@Autonomous(group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class Bot2_5 extends LinearOpMode {
    static boolean
            isParkedMiddle = true,
            isUnderTruss = false;

    public static double
            X_START_BOTTOM = -37;

    public static double
            BACKBOARD_X = 50;

    public static EditablePose
            botStartRed = new EditablePose(X_START_BOTTOM, -61.788975, BACKWARD),
            botLeftSpikeRed = new EditablePose(-49 , -16, LEFT),
            botCenterSpikeRed = new EditablePose(-50, -22, LEFT),
            botRightSpikeRed = new EditablePose(-33, -35, toRadians(210)),
            botCenterBackdropRed = new EditablePose(48, -35, LEFT),
            botLeftBackdropRed = new EditablePose(48, -29, LEFT),
            botRightBackdropRed = new EditablePose(48, -41, LEFT),
            botParkingLeftRed = new EditablePose(48, -10, LEFT),
            botParkingRightRed = new EditablePose(48, -60, LEFT),
            botMovement = new EditablePose(33,-11, LEFT),
            botAudienceSpikeTransitionRed = new EditablePose(-34,-18,toRadians(110)),
            botTrussSpikeTransitionRed = new EditablePose(-37,-43,toRadians(210)),
            botStageDoor = new EditablePose(25,-10,LEFT),
            botTrussInner = new EditablePose(20,-36,LEFT),
            botTrussOuter = new EditablePose(20,-58,LEFT),
            firstWhitePixelStackRed = new EditablePose(-58,-11, LEFT),
            trussTransition = new EditablePose(-53,-58,LEFT),
            thirdWhitePixelStackRed = new EditablePose(-58, -36,LEFT);

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
            mTelemetry.addLine("| B - RED | X - BLUE |");
            mTelemetry.addLine("| A - PARK MIDDLE | Y - PARK CORNER");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + " " + (isParkedMiddle ? "PARK MIDDLE" : "PARK CORNER"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }

        TrajectorySequence[] trajectories = {getTrajectory(0), getTrajectory(1), getTrajectory(2)};
        TrajectorySequence trajectory = null;

        propSensor = new PropSensor(hardwareMap, isRed);

        while (!propSensor.getIsOpened()) {
            mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isParkedMiddle ? "PARK MIDDLE" : "PARK CORNER"));
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

        robot.drivetrain.followTrajectorySequenceAsync(trajectory);

        while (opModeIsActive()) {
            if (!opModeIsActive()) {
                return;
            }

            robot.readSensors();

            robot.drivetrain.update();
            robot.run();
        }

        autonEndPose = robot.drivetrain.getPoseEstimate();

    }

    private TrajectorySequence getTrajectory(int randomization) {
        switch (randomization) {
            case 0:
                mainSpike = isRed ?  botLeftSpikeRed : botRightSpikeRed;
                yellowScoring = isRed ? botLeftBackdropRed : botRightBackdropRed;
                transition = isUnderTruss ? botTrussOuter : botStageDoor;
                pixelStack = isUnderTruss ? thirdWhitePixelStackRed : firstWhitePixelStackRed;
                whiteScoring = isRed ? botRightBackdropRed : botLeftBackdropRed;
                break;
            case 1:
                mainSpike = botCenterSpikeRed;
                yellowScoring = botCenterBackdropRed;
                transition = botTrussInner;
                pixelStack = thirdWhitePixelStackRed;
                whiteScoring = botRightBackdropRed;
                break;
            case 2:
                mainSpike = isRed ? botRightSpikeRed : botLeftSpikeRed;
                yellowScoring = isRed ? botRightBackdropRed : botLeftBackdropRed;
                transition = isUnderTruss ? botTrussOuter : botStageDoor;
                pixelStack = isUnderTruss ? thirdWhitePixelStackRed : firstWhitePixelStackRed;
                whiteScoring = isRed ? botLeftBackdropRed : botRightBackdropRed;
                break;
        }

        Pose2d startPose = botStartRed.byAlliancePose2d();
        robot.drivetrain.setPoseEstimate(startPose);
        TrajectorySequenceBuilder builder = robot.drivetrain.trajectorySequenceBuilder(startPose);

        scorePurplePixel(builder, randomization); // good
        getWhitePixels(builder, randomization); // good
        scoreYellowPixel(builder); // good
        getWhitePixels2(builder, randomization);
        scoreWhitePixels(builder, randomization);
        getWhitePixels2(builder, randomization);
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
            builder
                    .strafeRight(6);
        } else if (isCenter(randomization) || isBackboardSide(randomization)) {
            builder.lineToSplineHeading(mainSpike.byAlliancePose2d());
        }
        //.addTemporalMarker(() -> robot.purplePixel.setActivated(true));
    }

    private void getWhitePixels(TrajectorySequenceBuilder builder, int randomization) {
        builder
                .lineToSplineHeading(pixelStack.byAlliancePose2d());

        if (isUnderTruss && !isCenter(randomization)) builder.lineToConstantHeading(trussTransition.byAllianceVec());

    }
    private void scoreYellowPixel(TrajectorySequenceBuilder builder) {
        builder
                .setTangent(RIGHT)
                .splineTo(transition.byAllianceVec(), RIGHT)
                .splineToConstantHeading(yellowScoring.byAllianceVec(), LEFT);

        score(builder);
    }
    private void getWhitePixels2(TrajectorySequenceBuilder builder, int randomization) {
        builder.setTangent(LEFT);

        builder.splineToConstantHeading(transition.byAllianceVec(), LEFT);

        if (isUnderTruss && randomization != 1)
            builder.splineToConstantHeading(trussTransition.byAllianceVec(),LEFT)
                    .splineTo(pixelStack.byAllianceVec(), LEFT);

    }

    private void scoreWhitePixels(TrajectorySequenceBuilder builder, int randomization) {
        if (isUnderTruss && randomization != 1) builder.splineToConstantHeading(trussTransition.byAllianceVec(),LEFT);
        builder.setTangent(RIGHT);

//        if (isUnderTruss && randomization != 1) builder.splineTo(outerTruss2.byAllianceVec(), RIGHT);

        builder.splineTo(transition.byAllianceVec(), RIGHT)
                .splineToConstantHeading(whiteScoring.byAllianceVec(), RIGHT);

        score(builder);
    }

    private void score(TrajectorySequenceBuilder builder) {
        builder.addTemporalMarker(() -> robot.lift.setToAutonHeight(0))
                .waitSeconds(1)
                .addTemporalMarker(() -> robot.arm.setArm(true))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.arm.setFlap(false))
                .waitSeconds(2)
                .addTemporalMarker(() -> robot.lift.setToAutonHeight(400))
                .waitSeconds(2)
                .addTemporalMarker(() -> robot.arm.setArm(false))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.lift.retract());
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
