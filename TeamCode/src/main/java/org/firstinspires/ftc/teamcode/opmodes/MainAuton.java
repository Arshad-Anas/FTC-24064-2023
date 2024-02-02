package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.vision.PropSensor;

@Config
@Autonomous(group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {
    static Robot robot;
    static PropSensor propSensor;
    public static MultipleTelemetry mTelemetry;
    public static GamepadEx gamepadEx1, gamepadEx2;

    static boolean
            isRed = false,
            isTop = true;

    public static int propPlacement = 1;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static double
            BOTTOM_START_X = -37,
            BACKBOARD_X = 59;

    public static EditablePose
            // Bottom
            botStartRed = new EditablePose(BOTTOM_START_X, -61.788975, FORWARD),
            botLeftSpikeRed = new EditablePose(-51, -36.5, FORWARD),
            botCenterSpikeRed = new EditablePose(-41, -32, FORWARD),
            botRightSpikeRed = new EditablePose(-37, -35.5, FORWARD),
            botLeftPixelDodgeRed = new EditablePose(-36.5, -50, FORWARD),
            botCenterPixelDodgeRed = new EditablePose (-53, -38, FORWARD),
            botCenterPixelDodgeRed2 = new EditablePose(-53, -10, FORWARD),
            botStageDoorRed = new EditablePose(-36.5, -7, RIGHT),
            botTransitionRed = new EditablePose(40, -7, LEFT),
            botLeftBackdropRed = new EditablePose(BACKBOARD_X - 2, -26, LEFT),
            botCenterBackdropRed = new EditablePose(BACKBOARD_X + 2, -32, LEFT),
            botRightBackdropRed = new EditablePose(BACKBOARD_X - 3.5, -51, LEFT),
            // Top
            topLeftBackdropRed = new EditablePose(BACKBOARD_X - 7, -27, LEFT),
            topCenterBackdropRed = new EditablePose(BACKBOARD_X - 6, -31, LEFT),
            topRightBackdropRed = new EditablePose(BACKBOARD_X - 7, -38, LEFT),
            topParkingRed = new EditablePose(47.5, -60, LEFT),
            // Test RED
            testLeftBackdrop = new EditablePose(BACKBOARD_X + 4, -31, LEFT),
            testCenterBackdrop = new EditablePose(BACKBOARD_X + 4, -35, LEFT),
            testRightBackdrop = new EditablePose(BACKBOARD_X + 4, -44, LEFT),

            testLeftBackdropTop = new EditablePose(BACKBOARD_X - 5.5, -27, LEFT),
            testCenterBackdropTop = new EditablePose(BACKBOARD_X - 6, -31, LEFT),
            testRightBackdropTop = new EditablePose(BACKBOARD_X - 3, -38, LEFT);

    private static EditablePose prop, dodge, yellowPixel;

    public static Pose2d autonEndPose = null;

    public static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize gamepad (ONLY FOR INIT, DON'T CALL DURING WHILE LOOP)
        gamepadEx1 = new GamepadEx(gamepad1);

        robot = new Robot(hardwareMap);
        //propSensor = new PropSensor(hardwareMap, isRed);

        // Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (keyPressed(1, DPAD_UP)) isTop = true;
            if (keyPressed(1, DPAD_DOWN)) isTop = false;
            if (keyPressed(1, B)) isRed = true;
            if (keyPressed(1, X)) isRed = false;
            mTelemetry.addLine("| B - RED | X - BLUE |");
            mTelemetry.addLine("| D-pad-down - BOTTOM | D-pad-up - TOP |");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + " " + (isTop ? "TOP" : "BOTTOM"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }

        propSensor = new PropSensor(hardwareMap, isRed);

        while (!propSensor.getIsOpened()) {
            mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isTop ? "TOP" : "BOTTOM"));
            mTelemetry.addLine("Camera is not open");
            mTelemetry.update();
        }

        while (!isStarted() && !isStopRequested()) {
            propPlacement = propSensor.propPosition();
            mTelemetry.addData("Predicted Prop Placement", propPlacement);
            mTelemetry.update();
            sleep(50);
        }

        propSensor.getCamera().stopStreaming();
        propSensor.getCamera().closeCameraDevice();

        robot.drivetrain.followTrajectorySequenceAsync(getTrajectory());

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

    private TrajectorySequence getTrajectory() {
        switch (propPlacement) {
            case 0:
                prop = isTop ? (isRed ? botRightSpikeRed : botLeftSpikeRed) : (isRed ? botLeftSpikeRed : botRightSpikeRed);
                dodge = botLeftPixelDodgeRed;
                yellowPixel = isTop ? (isRed ? testLeftBackdropTop : topRightBackdropRed) : (isRed ? testLeftBackdrop : botRightBackdropRed);
                break;
            case 1:
                prop = botCenterSpikeRed;
                dodge = botCenterPixelDodgeRed;
                yellowPixel = isTop ? (isRed ? testCenterBackdropTop : topCenterBackdropRed) : (isRed ? testCenterBackdrop : botCenterBackdropRed);
                break;
            case 2:
                prop = isTop ? (isRed ? botLeftSpikeRed : botRightSpikeRed) : (isRed ? botRightSpikeRed : botLeftSpikeRed);
                dodge = botLeftPixelDodgeRed;
                yellowPixel = isTop ? (isRed ? testRightBackdropTop : topLeftBackdropRed) : (isRed ? testRightBackdrop : botLeftBackdropRed);
                break;
        }

        Pose2d start = botStartRed.bySide().byAlliancePose2d();
        robot.drivetrain.setPoseEstimate(start);

        TrajectorySequenceBuilder builder = robot.drivetrain.trajectorySequenceBuilder(start);

        addPurplePixel(builder);
        addYellowPixel(builder);

        return builder.build();
    }

    private void addYellowPixel(TrajectorySequenceBuilder builder) {
        if (!isTop) {
            if (isLeft() || isRight())
                builder.lineTo(botStageDoorRed.byAllianceVec());
            else
                builder.splineToConstantHeading(botStageDoorRed.byAllianceVec(), RIGHT);

            builder.lineToSplineHeading(botTransitionRed.byAlliancePose2d());
        }

        // Scoring
        builder.lineToSplineHeading(yellowPixel.byAlliancePose2d())
                .addTemporalMarker(() -> robot.lift.setToAutonHeight()) // Lift and arm extend
                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.arm.setFlap(false))
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    // Go up to auton 2nd height
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> robot.arm.toggleArm());

        // Parking for top
        if (isTop)
            builder.forward(3)
                    .lineToSplineHeading(topParkingRed.byAlliancePose2d())
                    .back(13);
    }

    private void addPurplePixel(TrajectorySequenceBuilder builder) {
        builder.lineToSplineHeading(prop.bySide().byAlliancePose2d());

        if (!isTop) {
            if (isLeft() || isCenter())
                builder.addTemporalMarker(() -> robot.intake.set(0.30))
                        .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.intake.set(0))
                        .back(11)
                        .lineToSplineHeading(dodge.byAlliancePose2d());

            if (isCenter())
                builder.lineToSplineHeading(botCenterPixelDodgeRed2.byAlliancePose2d());

            if (isRight())
                builder.turn(toRadians(isRed ? -90 : 90))
                        .forward(3)
                        .addTemporalMarker(() -> robot.intake.set(0.30))
                        .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.intake.set(0))
                        .waitSeconds(0.5)
                        .back(8)
                        .turn(toRadians(180));
        } else {
            if (isLeft())
                builder.turn(toRadians(isRed ? 90 : -90))
                        .forward(3)
                        .addTemporalMarker(() -> robot.intake.set(0.30))
                        .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.intake.set(0))
                        .waitSeconds(0.5);
            else
                builder.addTemporalMarker(() -> robot.intake.set(0.30))
                        .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.intake.set(0))
                        .back(11);
        }
    }

    private boolean isCenter() {
        return propPlacement == 1;
    }

    // Red-centric
    private boolean isLeft() {
        return isRed && propPlacement == 0 || !isRed && propPlacement == 2;
    }

    // Red-centric
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
            if (isTop) x = x * -1 - 23.5;
            return this;
        }
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