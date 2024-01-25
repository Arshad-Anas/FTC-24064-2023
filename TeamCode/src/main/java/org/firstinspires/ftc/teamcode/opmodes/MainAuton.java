package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;

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
            isParkedMiddle = true,
            isTop = true;

    public static int propPlacement = 1;

    public static double
            X_START_BOTTOM = -37,
            X_START_TOP = 12;

    // Top constants
    public static double
            OPENING_SLIDE_TIME = 1.25,
            OPEN_FLAP_WAIT_TIME = 0.25,
            SCORING_WAIT_TIME = 1;

    // Bottom constants
    public static double
            TIME_START_INTAKE = 5,
            TIME_START_OUTTAKE = 9,
            TIME_STOP_OUTTAKE = 12,
            TIME_LIFT_SLIDES = 15;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static double
            BACKBOARD_X = 52.18;

    // Bottom
    public static EditablePose
            botStartRed = new EditablePose(X_START_BOTTOM, -61.788975, FORWARD),
            botLeftSpikeRed = new EditablePose(-51, -39, FORWARD),
            botCenterSpikeRed = new EditablePose(-41, -32, FORWARD),
            botRightSpikeRed = new EditablePose(-37, -34, FORWARD),
            botLeftPixelDodgeRed = new EditablePose(-42, -50, FORWARD),
            botCenterPixelDodgeRed = new EditablePose (-53, -38, FORWARD),
            botCenterPixelDodgeRed2 = new EditablePose(-53, -20, FORWARD),
            botStageDoorRed = new EditablePose(-30, -9, RIGHT),
            botTransitionRed = new EditablePose(25, -9, LEFT),
            botLeftBackdropRed = new EditablePose(BACKBOARD_X, -30, LEFT),
            botCenterBackdropRed = new EditablePose(BACKBOARD_X, -36.5, LEFT),
            botRightBackdropRed = new EditablePose(BACKBOARD_X, -43, LEFT);

    private static EditablePose start, prop, dodge, yellowPixel;

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
            if (keyPressed(1, A)) isParkedMiddle = true;
            if (keyPressed(1, Y)) isParkedMiddle = false;
            mTelemetry.addLine("| B - RED | X - BLUE |");
            mTelemetry.addLine("| A - PARK MIDDLE | Y - PARK CORNER");
            mTelemetry.addLine("| D-pad-down - BOTTOM | D-pad-up - TOP |");
            mTelemetry.addLine();
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + " " + (isTop ? "TOP" : "BOTTOM") + " " + (isParkedMiddle ? "PARK MIDDLE" : "PARK CORNER"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }

        propSensor = new PropSensor(hardwareMap, isRed);

        while (!propSensor.getIsOpened()) {
            mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isTop ? "TOP" : "BOTTOM") + " " + (isParkedMiddle ? "PARK MIDDLE" : "PARK CORNER"));
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
                yellowPixel = isRed ? botLeftBackdropRed : botRightBackdropRed;
                break;
            case 1:
                prop = botCenterSpikeRed;
                dodge = botCenterPixelDodgeRed;
                yellowPixel = botCenterBackdropRed;
                break;
            case 2:
                prop = isTop ? (isRed ? botLeftSpikeRed : botRightSpikeRed) : (isRed ? botRightSpikeRed : botLeftSpikeRed);
                dodge = botLeftPixelDodgeRed;
                yellowPixel = isRed ? botRightBackdropRed : botLeftBackdropRed;
                break;
        }

        start = botStartRed;
        robot.drivetrain.setPoseEstimate(start.bySide().byAlliancePose2d());

        TrajectorySequenceBuilder builder = robot.drivetrain.trajectorySequenceBuilder(start.bySide().byAlliancePose2d());

        addPurplePixel(builder);

        if (!isTop)
            builder.splineToConstantHeading(botStageDoorRed.byAllianceVec(), RIGHT)
                .lineToSplineHeading(botTransitionRed.byAlliancePose2d())
                .lineToSplineHeading(yellowPixel.byAlliancePose2d())
                .addTemporalMarker(() -> robot.lift.setToAutonHeight()) // Lift and arm extend
                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.arm.setFlap(false))
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(2.2, () -> robot.arm.toggleArm());

        return builder.build();
    }

    private void addPurplePixel(TrajectorySequenceBuilder builder) {
        builder.lineToSplineHeading(prop.bySide().byAlliancePose2d());

        if (!isTop) {
            if (isLeft() || isCenter())
                builder.addTemporalMarker(() -> robot.intake.set(0.25))
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.intake.set(0))
                        .back(11)
                        .lineToSplineHeading(dodge.byAlliancePose2d());

            if (isCenter())
                builder.lineToSplineHeading(botCenterPixelDodgeRed2.byAlliancePose2d());

            if (isRight())
                builder.turn(toRadians(isRed ? -90 : 90))
                        .addTemporalMarker(() -> robot.intake.set(0.25))
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.intake.set(0))
                        .turn(toRadians(isRed ? 90 : -90));
        } else {
            if (isLeft())
                builder.turn(toRadians(isRed ? 90 : -90))
                        .addTemporalMarker(() -> robot.intake.set(0.25))
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.intake.set(0))
                        .turn(toRadians(isRed ? -90 : 90));
            else
                builder.addTemporalMarker(() -> robot.intake.set(0.25))
                        .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.intake.set(0));
        }
    }

    private boolean isCenter() {
        return propPlacement == 1;
    }

    private boolean isLeft() {
        return isRed && propPlacement == 0 || !isRed && propPlacement == 2;
    }

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