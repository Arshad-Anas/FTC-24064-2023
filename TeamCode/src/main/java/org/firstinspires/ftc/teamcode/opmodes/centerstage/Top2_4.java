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
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.Top2_4.TrajectoryState.GETTING_WHITE;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.Top2_4.TrajectoryState.GETTING_WHITE_2;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.Top2_4.TrajectoryState.PARKING;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.Top2_4.TrajectoryState.READING_APRIL_TAG;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.Top2_4.TrajectoryState.SCORING_PURPLE;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.Top2_4.TrajectoryState.SCORING_WHITE;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.Top2_4.TrajectoryState.SCORING_YELLOW;
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
import org.firstinspires.ftc.teamcode.subsystems.drivetrains.MecanumDrivetrain;

@Config
@Autonomous(name = "Top 2+4", group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class Top2_4 extends LinearOpMode {
    static boolean
            isParkedMiddle = true,
            isUnderTruss = false,
            doAprilTag = false;

    public static double
            START_X = 12,
            BACKBOARD_X = 51.6,
            ANGLE_1 = 52,
            ANGLE_2 = 46,
            ANGLE_3 = 32.5,
            ANGLE_4 = 20;

    public static EditablePose
            start = new EditablePose(START_X, -61.788975, BACKWARD),
            spikeLeftBlue = new EditablePose((START_X - 6), -34.5, toRadians(135)),
            spikeCenterBlue = new EditablePose((START_X + 6), -26, toRadians(315)),
            spikeRightBlue = new EditablePose(30, -36, toRadians(315)),
            spikeLeftRed = new EditablePose(3, -35, toRadians(135)),
            spikeCenterRed = new EditablePose(24, -27, RIGHT),
            spikeRightRed = new EditablePose(START_X + 14, -34.5, toRadians(315)),
            backboardLeft = new EditablePose(BACKBOARD_X, -30.5, LEFT),
            backboardCenter = new EditablePose(BACKBOARD_X, -34.5, LEFT),
            backboardRight = new EditablePose(BACKBOARD_X, -41, LEFT),
            parkingLeft = new EditablePose(48.5, -10, toRadians(165)),
            parkingRight = new EditablePose(48.5, -56, toRadians(200)),
            spikeDodgeStageDoor = new EditablePose(23, -10, LEFT),
            stageDoor = new EditablePose(13, -10, LEFT),
            innerTruss = new EditablePose(-8, -34.5, LEFT),
            outerTruss = new EditablePose(23.5, -58, LEFT),
            outerTruss2 = new EditablePose(-23.5, -58, LEFT),
            pixelStack1 = new EditablePose(-56.6, -12, LEFT),
            pixelStack3 = new EditablePose(-56.6, -35, LEFT);

    private EditablePose mainSpike, pixelStack, whiteScoring, yellowScoring, transition;
    private MecanumDrivetrain drive;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize gamepad (ONLY FOR INIT, DON'T CALL DURING WHILE LOOP)
        gamepadEx1 = new GamepadEx(gamepad1);

        robot = new Robot(hardwareMap);
        drive = robot.drivetrain;

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
        TrajectorySequence[] scorePurplePixelArr, scoreYellowPixelArr, getWhitePixelArr, scoreWhitePixelArr, getWhitePixel2Arr, parkingArr;
        scorePurplePixelArr = scoreYellowPixelArr = getWhitePixelArr = scoreWhitePixelArr = getWhitePixel2Arr = parkingArr = new TrajectorySequence[3];

        for (int i = 1; i <= 3; i++) {
            getTrajectory(i, scorePurplePixelArr[i], scoreYellowPixelArr[i], getWhitePixelArr[i], scoreWhitePixelArr[i], getWhitePixel2Arr[i], parkingArr[i]);
        }

        propSensor = new PropSensor(hardwareMap, isRed);

        while (!propSensor.getIsOpened()) {
            mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isParkedMiddle ? "PARK MIDDLE" : "PARK CORNER") + " " + (isUnderTruss ? "UNDER TRUSS" : "UNDER DOOR") + " " + (doAprilTag ? "APRIL TAG" : "NO APRIL TAG"));
            mTelemetry.addLine("Camera is not open");
            mTelemetry.update();
        }

        int randomization = 0;
        while (!isStarted() && !isStopRequested()) {
            randomization = propSensor.propPosition();
            mTelemetry.addData("Predicted Prop Placement", randomization);
            mTelemetry.update();
            sleep(50);
        }

//        propSensor.getCamera().stopStreaming();
        propSensor.getCamera().closeCameraDeviceAsync(() -> {
            mTelemetry.addLine("Camera closed");
            mTelemetry.update();
        });

        TrajectorySequence scorePurplePixel, scoreYellowPixel, getWhitePixel, scoreWhitePixel, getWhitePixel2, parking;
        scorePurplePixel = scorePurplePixelArr[randomization];
        scoreYellowPixel = scoreYellowPixelArr[randomization];
        getWhitePixel = getWhitePixelArr[randomization];
        scoreWhitePixel = scoreWhitePixelArr[randomization];
        getWhitePixel2 = getWhitePixel2Arr[randomization];
        parking = parkingArr[randomization];

        aprilTag = new AprilTagLocalization(hardwareMap);
        int cycles = 0;

        TrajectoryState state = SCORING_PURPLE;

        while (opModeIsActive()) {
            if (!opModeIsActive()) {
                return;
            }

            robot.readSensors();

            switch (state) {
                case SCORING_PURPLE:
                    if (!drive.isBusy()) {
                        drive.setPoseEstimate(start.byAlliancePose2d());
                        drive.followTrajectorySequenceAsync(scorePurplePixel);
                        state = SCORING_YELLOW;
                    }
                case SCORING_YELLOW:
                    if (!drive.isBusy()) {
                        drive.setPoseEstimate(scorePurplePixel.end());
                        drive.followTrajectorySequenceAsync(scoreYellowPixel);
                        state = READING_APRIL_TAG;
                    }
                case GETTING_WHITE:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(getWhitePixel);
                        state = SCORING_WHITE;
                    }
                case SCORING_WHITE:
                    if (!drive.isBusy()) {
                        drive.setPoseEstimate(getWhitePixel.end());
                        drive.followTrajectorySequenceAsync(scoreWhitePixel);
                        state = READING_APRIL_TAG;
                        cycles++;
                    }
                case GETTING_WHITE_2:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(getWhitePixel2);
                        state = SCORING_WHITE;
                    }
                case PARKING:
                    if (!drive.isBusy()) {
                        drive.followTrajectorySequenceAsync(parking);
                    }
                case READING_APRIL_TAG:
                    if (!drive.isBusy()) {
                        Pose2d estimate = aprilTag.getPoseEstimate();
                        if (estimate != null) drive.setPoseEstimate(estimate);

                        if (cycles == 0) state = GETTING_WHITE;
                        else if (cycles == 1) state = GETTING_WHITE_2;
                        else state = PARKING;
                    }
            }

            robot.drivetrain.update();
            robot.run();
        }

        autonEndPose = robot.drivetrain.getPoseEstimate();
    }

    private void getTrajectory(int randomization, TrajectorySequence scorePurplePixel, TrajectorySequence scoreYellowPixel, TrajectorySequence getWhitePixels, TrajectorySequence scoreWhitePixels, TrajectorySequence getWhitePixels2, TrajectorySequence parking) {
        switch (randomization) {
            case 0:
                mainSpike = isRed ? spikeLeftRed : spikeRightBlue;
                yellowScoring = isRed ? backboardLeft : backboardRight;
                transition = isRed ? (isUnderTruss ? outerTruss : stageDoor) : (isUnderTruss ? outerTruss : spikeDodgeStageDoor);
                pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
                whiteScoring = isUnderTruss ? backboardRight : backboardCenter;
                break;
            case 1:
                mainSpike = isRed ? spikeCenterRed : spikeCenterBlue;
                yellowScoring = backboardCenter;
                transition = isRed ? (isUnderTruss ? innerTruss : stageDoor) : (isUnderTruss ? innerTruss : spikeDodgeStageDoor);
                pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
                whiteScoring = isUnderTruss ? backboardRight : backboardLeft;
                break;
            case 2:
                mainSpike = isRed ? spikeRightRed : spikeLeftBlue;
                yellowScoring = isRed ? backboardRight : backboardLeft;
                transition = isRed ? (isUnderTruss ? outerTruss : spikeDodgeStageDoor) : (isUnderTruss ? outerTruss : stageDoor);
                pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
                whiteScoring = isUnderTruss ? backboardRight : backboardLeft;
                break;
        }


        scorePurplePixel = scorePurplePixel(start.byAlliancePose2d(), randomization);
        scoreYellowPixel = scoreYellowPixel(scorePurplePixel.end());
        getWhitePixels = getWhitePixels(scoreYellowPixel.end(), randomization, 1);
        scoreWhitePixels = scoreWhitePixels(getWhitePixels.end(), randomization);
        getWhitePixels2 = getWhitePixels(scoreWhitePixels.end(), randomization, 2);
        parking = drive.trajectorySequenceBuilder(scoreWhitePixels.end()).lineToSplineHeading((isParkedMiddle ? parkingLeft : parkingRight).byAlliancePose2d()).build();
    }

    private TrajectorySequence scorePurplePixel(Pose2d start, int randomization) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);

        if (isBackboardSide(randomization) || randomization == 1) {
            builder.lineToSplineHeading(mainSpike.byAlliancePose2d());
        } else {
            builder.setTangent(isRed ? FORWARD : BACKWARD)
                    .splineTo(mainSpike.byAllianceVec(), mainSpike.byAlliance().heading);
        }

        builder.addTemporalMarker(() -> robot.purplePixel.setActivated(true));
        return builder.build();
    }

    private TrajectorySequence scoreYellowPixel(Pose2d start) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);

        builder.lineToSplineHeading(yellowScoring.byAlliancePose2d());
        setSlides(builder, false);
        score(builder);
        return builder.build();
    }

    private TrajectorySequence getWhitePixels(Pose2d start, int randomization, int cycle) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);

        retractSlides(builder);

        builder.setTangent(LEFT)
                .splineTo(transition.byAllianceVec(), transition.heading);

        if (isUnderTruss && randomization != 1) builder.splineTo(outerTruss2.byAllianceVec(), outerTruss2.heading);

        builder.splineTo(pixelStack.byAllianceVec(), pixelStack.heading);
        intakePixels(builder, cycle);
        return builder.build();
    }

    private TrajectorySequence scoreWhitePixels(Pose2d start, int randomization) {
        TrajectorySequenceBuilder builder = drive.trajectorySequenceBuilder(start);

        builder.setTangent(RIGHT);

        if (isUnderTruss && randomization != 1) builder.splineTo(outerTruss2.byAllianceVec(), RIGHT);

        builder.splineTo(transition.byAllianceVec(), RIGHT);
        setSlides(builder, true);
        builder.splineTo(whiteScoring.byAllianceVec(), RIGHT);

        score(builder);
        retractSlides(builder);
        return builder.build();
    }

    private void intakePixels(TrajectorySequenceBuilder builder, int cycle) {
        builder.addTemporalMarker(() -> {
            robot.rollers.setDeployable(cycle == 1 ? ANGLE_1 : ANGLE_3);
            robot.rollers.intake(-1);
        })
                .waitSeconds(0.8)
                .addTemporalMarker(() -> robot.rollers.setDeployable(cycle == 1 ? ANGLE_2 : ANGLE_4))
                .waitSeconds(0.8)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.rollers.resetDeployable())
                .UNSTABLE_addTemporalMarkerOffset(2, () -> robot.rollers.intake(0));
    }

    private void score(TrajectorySequenceBuilder builder) {
        builder.addTemporalMarker(() -> {
                    robot.arm.setArm(true);
                    robot.wrist.setActivated(true);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> robot.arm.setFlap(false))
                .waitSeconds(0.4)
                .addTemporalMarker(() -> {
                    robot.arm.setArm(false);
                    robot.wrist.setActivated(false);
                });
    }

    private void setSlides(TrajectorySequenceBuilder builder, boolean isWhite) {
        builder.UNSTABLE_addTemporalMarkerOffset(0.75, () -> robot.lift.setToAutonHeight(isWhite ? 700 : 400));
    }

    private void retractSlides(TrajectorySequenceBuilder builder) {
        builder.UNSTABLE_addTemporalMarkerOffset(0.75, () -> robot.lift.retract());
    }

    private boolean isBackboardSide(int randomization) {
        return isRed && randomization == 2 || !isRed && randomization == 0;
    }

    enum TrajectoryState {
        SCORING_PURPLE,
        SCORING_YELLOW,
        GETTING_WHITE,
        SCORING_WHITE,
        GETTING_WHITE_2,
        PARKING,
        READING_APRIL_TAG;
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
            EditablePose pose = new EditablePose(x, y, heading);
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