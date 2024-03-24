package org.firstinspires.ftc.teamcode.opmode.centerstage;

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

import static org.firstinspires.ftc.teamcode.opmode.centerstage.MainAuton.BACKWARD;
import static org.firstinspires.ftc.teamcode.opmode.centerstage.MainAuton.FORWARD;
import static org.firstinspires.ftc.teamcode.opmode.centerstage.MainAuton.LEFT;
import static org.firstinspires.ftc.teamcode.opmode.centerstage.MainAuton.aprilTag;
import static org.firstinspires.ftc.teamcode.opmode.centerstage.MainAuton.autonEndPose;
import static org.firstinspires.ftc.teamcode.opmode.centerstage.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmode.centerstage.MainAuton.isRed;
import static org.firstinspires.ftc.teamcode.opmode.centerstage.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmode.centerstage.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmode.centerstage.MainAuton.propSensor;
import static org.firstinspires.ftc.teamcode.opmode.centerstage.MainAuton.robot;
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
import org.firstinspires.ftc.teamcode.subsystems.utilities.vision.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.subsystems.utilities.vision.PropSensor;

@Config
@Autonomous(name = "Bottom 2+5", group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class Bot2_5 extends LinearOpMode {
    static boolean
            isParkedMiddle = true,
            isUnderTruss = false,
            doAprilTag = false;

    public static double
            START_X = -35.5;

    public static double
            BACKBOARD_X = 51.6,
            ANGLE_1 = 52,
            ANGLE_2 = 46,
            ANGLE_3 = 32.5,
            ANGLE_4 = 20,
            ANGLE_5 = 2;

    public static EditablePose
            botStartRed = new EditablePose(START_X, -61.788975, BACKWARD),
            leftSpikeRed = new EditablePose(START_X , -15, FORWARD),
            leftSpikeRed2 = new EditablePose(START_X - 14 , leftSpikeRed.y, FORWARD),
            centerSpikeRed = new EditablePose(-50, -22, LEFT),
            centerSpikeBlue = new EditablePose(-50 , -27, LEFT),
            rightSpikeRed = new EditablePose(START_X + 6, -36.5, toRadians(45)),
            rightSpikeBlue = new EditablePose(START_X + 6, -36.5, toRadians(325)),
            backboardCenter = new EditablePose(BACKBOARD_X, -34.5, LEFT),
            backboardLeft = new EditablePose(BACKBOARD_X, -30.5, LEFT),
            backboardRight = new EditablePose(BACKBOARD_X, -41, LEFT),
            spikeDodgeStageDoor = new EditablePose(28, -11.5, LEFT),
            outerTruss = new EditablePose(20,-58,LEFT),
            outerTruss2 = new EditablePose(-23.5, -58, LEFT),
            pixelStack1 = new EditablePose(-59,-11.5, LEFT),
            trussTransition = new EditablePose(-53,-58,LEFT),
            pixelStack3 = new EditablePose(-59, -35,LEFT);

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
                mainSpike = isRed ? leftSpikeRed : rightSpikeBlue;
                yellowScoring = isRed ? backboardLeft : backboardRight;
                transition = isUnderTruss ? outerTruss : spikeDodgeStageDoor;
                pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
                whiteScoring = isUnderTruss ? backboardRight : backboardLeft;
                break;
            case 1:
                mainSpike = isRed ? centerSpikeRed : centerSpikeBlue;
                yellowScoring = backboardCenter;
                transition = isUnderTruss ? outerTruss : spikeDodgeStageDoor;
                pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
                whiteScoring = isUnderTruss ? backboardCenter : backboardLeft;
                break;
            case 2:
                mainSpike = isRed ? rightSpikeRed : leftSpikeRed;
                yellowScoring = isRed ? backboardRight : backboardLeft;
                transition = isUnderTruss ? outerTruss : spikeDodgeStageDoor;
                pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
                whiteScoring = isUnderTruss ? backboardRight : backboardLeft;
                break;
        }

        Pose2d startPose = botStartRed.byAlliancePose2d();
        robot.drivetrain.setPoseEstimate(startPose);
        TrajectorySequenceBuilder builder = robot.drivetrain.trajectorySequenceBuilder(startPose);

        scorePurplePixel(builder, randomization);
        getFirstWhitePixel(builder, randomization);
        scoreYellowPixel(builder);
        getWhitePixels(builder ,1);
        scoreWhitePixels(builder);

        return builder.build();
    }

    private void scorePurplePixel(TrajectorySequenceBuilder builder, int randomization) {
        builder.setTangent(isRed ? FORWARD : BACKWARD);
        if (isBackboardSide(randomization)) {
            builder.splineTo(mainSpike.byAllianceVec(), mainSpike.heading);
        } else if (isAudienceSide(randomization)) {
            builder.lineToSplineHeading(mainSpike.byAlliancePose2d())
                    .lineTo(leftSpikeRed2.byAllianceVec());
        } else {
            builder.lineToSplineHeading(mainSpike.byAlliancePose2d());
        }
        builder.addTemporalMarker(() -> robot.purplePixel.setActivated(true));
    }

    private void getFirstWhitePixel(TrajectorySequenceBuilder builder, int randomization) {
        builder.lineToSplineHeading(pixelStack.byAlliancePose2d());

        intakePixels(builder, 0);

        if (isUnderTruss && !isCenter(randomization)) builder.lineToConstantHeading(trussTransition.byAllianceVec());
    }

    private void scoreYellowPixel(TrajectorySequenceBuilder builder) {
        builder.lineTo(transition.byAllianceVec())
                .lineTo(yellowScoring.byAllianceVec());

        score(builder, true);
    }

    private void getWhitePixels(TrajectorySequenceBuilder builder, int cycle) {
        builder.lineTo(transition.byAllianceVec());

        if (isUnderTruss) builder.lineTo(outerTruss2.byAllianceVec());

        builder.lineTo(pixelStack.byAllianceVec());
        intakePixels(builder, cycle);
    }

    private void scoreWhitePixels(TrajectorySequenceBuilder builder) {
        if (isUnderTruss) builder.lineTo(outerTruss2.byAllianceVec());

        builder.lineTo(transition.byAllianceVec())
                .lineTo(whiteScoring.byAllianceVec());

        score(builder, true);
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

    private static boolean isCenter(int randomization) {
        return randomization == 1;
    }

    private static boolean isAudienceSide(int randomization) {
        return isRed && randomization == 0 || !isRed && randomization == 2;
    }

    private static boolean isBackboardSide(int randomization) {
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