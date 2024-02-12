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
import org.firstinspires.ftc.teamcode.subsystems.centerstage.vision.PropSensor;

@Config
@Autonomous(group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class Top2_4 extends LinearOpMode {
    static boolean
            isParkedMiddle = true,
            isUnderTruss = false;

    public static double
            START_X = 13.5;

    public static double
            BACKBOARD_X = 50;

    public static EditablePose
            start = new EditablePose(START_X, -61.788975, BACKWARD),
            spikeLeft = new EditablePose((START_X - 6), -34.5, toRadians(135)),
            spikeCenter = new EditablePose((START_X + 6), -30, toRadians(120)),
            spikeRight = new EditablePose(START_X + 15, -40, toRadians(135)),
            backboardLeft = new EditablePose(BACKBOARD_X, -30.5, LEFT),
            backboardCenter = new EditablePose(BACKBOARD_X, -34.5, LEFT),
            backboardRight = new EditablePose(BACKBOARD_X, -41, LEFT),
            parkingLeft = new EditablePose(48.5, -10, toRadians(165)),
            parkingRight = new EditablePose(48.5, -56, toRadians(200)),
            stageDoor = new EditablePose(13, -10, LEFT),
            innerTruss = new EditablePose(-8, -34.5, LEFT),
            outerTruss = new EditablePose(23.5, -58, LEFT),
            outerTruss2 = new EditablePose(-23.5, -58, LEFT),
            pixelStack1 = new EditablePose(-58, -12, LEFT),
            pixelStack3 = new EditablePose(-58, -35, LEFT);

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
                mainSpike = isRed ? spikeLeft : spikeRight;
                yellowScoring = isRed ? backboardLeft : backboardRight;
                transition = isUnderTruss ? outerTruss : stageDoor;
                pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
                whiteScoring = isRed ? backboardRight : backboardLeft;
                break;
            case 1:
                mainSpike = spikeCenter;
                yellowScoring = backboardCenter;
                transition = innerTruss;
                pixelStack = pixelStack3;
                whiteScoring = backboardRight;
                break;
            case 2:
                mainSpike = isRed ? spikeRight : spikeLeft;
                yellowScoring = isRed ? backboardRight : backboardLeft;
                transition = isUnderTruss ? outerTruss : stageDoor;
                pixelStack = isUnderTruss ? pixelStack3 : pixelStack1;
                whiteScoring = isRed ? backboardLeft : backboardRight;
                break;
        }

        Pose2d startPose = start.byAlliancePose2d();
        robot.drivetrain.setPoseEstimate(startPose);
        TrajectorySequenceBuilder builder = robot.drivetrain.trajectorySequenceBuilder(startPose);

        scorePurplePixel(builder);
        scoreYellowPixel(builder);
        getWhitePixels(builder, randomization);
        scoreWhitePixels(builder, randomization);
        getWhitePixels(builder, randomization);
        scoreWhitePixels(builder, randomization);

        builder.lineToSplineHeading((isParkedMiddle ? parkingLeft : parkingRight).byAlliancePose2d());

        return builder.build();
    }

    private void scorePurplePixel(TrajectorySequenceBuilder builder) {
        builder.setTangent(isRed ? FORWARD : BACKWARD)
                .splineTo(mainSpike.byAllianceVec(), mainSpike.heading);
        //.addTemporalMarker(() -> robot.purplePixel.setActivated(true));
    }

    private void scoreYellowPixel(TrajectorySequenceBuilder builder) {
        builder.lineToSplineHeading(yellowScoring.byAlliancePose2d());
        score(builder);
    }

    private void getWhitePixels(TrajectorySequenceBuilder builder, int randomization) {
        builder.setTangent(LEFT)
                .splineTo(transition.byAllianceVec(), transition.heading);

        if (isUnderTruss && randomization != 1) builder.splineTo(outerTruss2.byAllianceVec(), outerTruss2.heading);

        builder.splineTo(pixelStack.byAllianceVec(), pixelStack.heading);
    }

    private void scoreWhitePixels(TrajectorySequenceBuilder builder, int randomization) {
        builder.setTangent(RIGHT);

        if (isUnderTruss && randomization != 1) builder.splineTo(outerTruss2.byAllianceVec(), RIGHT);

        builder.splineTo(transition.byAllianceVec(), RIGHT)
                .splineTo(whiteScoring.byAllianceVec(), RIGHT);

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