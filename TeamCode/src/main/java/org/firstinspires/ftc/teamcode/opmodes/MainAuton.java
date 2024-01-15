package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
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

@Config
@Autonomous(group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {
    static Robot robot;
    public static MultipleTelemetry mTelemetry;
    public static GamepadEx gamepadEx1, gamepadEx2;

    static boolean
            isRed = false,
            isParkedLeft = true,
            isTop = true;

    public static int propPlacement = 2;

    public static double
            X_START_BOTTOM = -35,
            X_START_TOP = 12,
            OUTTAKE_WAIT_TIME = 0.25,
            SCORING_WAIT_TIME = 0.75,
            OPEN_FLAP_WAIT_TIME = 0.25;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static EditablePose
            topStartRed = new EditablePose(X_START_TOP, -61.788975, FORWARD),
            topStartBlue = new EditablePose(topStartRed.byAlliance().x, 61, BACKWARD),
            topCenterSpikeRed = new EditablePose((X_START_TOP + 3.5), -33.5, FORWARD),
            topCenterSpikeBlue = new EditablePose(topCenterSpikeRed.byAlliance().x, 33.5, BACKWARD),
            topLeftSpikeRed = new EditablePose(7, -41, toRadians(120)),
            topLeftSpikeBlue = new EditablePose(topLeftSpikeRed.byAlliance().x, 41, toRadians(-120)),
            topRightSpikeRed = new EditablePose(24 - topLeftSpikeRed.x, topLeftSpikeRed.y, LEFT - topLeftSpikeRed.heading),
            topRightSpikeBlue = new EditablePose(topRightSpikeRed.byAlliance().x, 41, LEFT + topLeftSpikeRed.heading),
            topRedBackboard = new EditablePose(48, -34, RIGHT),
            topBlueBackboard = new EditablePose(topRedBackboard.byAlliance().x, 34, RIGHT),
            topRedParkingLeft = new EditablePose(52, -14, toRadians(165)),
            topRedParkingRight = new EditablePose(51, -54, toRadians(200)),
            topBlueParkingLeft = new EditablePose(topRedParkingLeft.x, 60, toRadians(165)),
            topBlueParkingRight = new EditablePose(topRedParkingRight.x, 14, toRadians(200));

    // Bottom
    private static final EditablePose
            // This is red alliance
            botStartPoseRed = new EditablePose(X_START_BOTTOM, -61.788975, 0),
            botCenterSpikeRed = new EditablePose(-39, -38, Math.toRadians(90)),
            botLeftSpikeRed = new EditablePose(-46.5, -47, toRadians(90)),
            botRightSpikeRed = new EditablePose(11 + botLeftSpikeRed.x, 13 + botLeftSpikeRed.y, Math.toRadians(0)),
            botWhitePixelRed = new EditablePose(-58, -24, Math.toRadians(-180)),
            botStageDoorRed = new EditablePose(-25, -10, Math.toRadians(0)), //15 could also work here if dBRed is removed, but this require further testing
            botTransitionRed = new EditablePose(25, -9, Math.toRadians(0)),
            botCenterBackdropRed = new EditablePose(49, -35, Math.toRadians(-180)),
            botLeftBackdropRed = new EditablePose(49, -30, Math.toRadians(-180)),
            botRightBackdropRed = new EditablePose(49, -41, Math.toRadians(-180)),
            botParkingLeftRed = new EditablePose(50, -10, Math.toRadians(-180)),
            botParkingRightRed = new EditablePose(50, -60, Math.toRadians(-180)),
            botLeftSpikeMovementRed = new EditablePose(-55, -45, Math.toRadians(180)),

            // This is for blue alliance
            botStartBlue = new EditablePose(botStartPoseRed.byAlliance().x, 61.788975, 0),
            botCenterSpikeBlue = new EditablePose(botCenterSpikeRed.byAlliance().x, 38, Math.toRadians(-90)),
            botRightSpikeBlue = new EditablePose(botLeftSpikeRed.byAlliance().x, 47, toRadians(-90)),
            botLeftSpikeBlue = new EditablePose(botRightSpikeRed.byAlliance().x, 34, Math.toRadians(0)),
            botWhitePixelBlue = new EditablePose(botWhitePixelRed.byAlliance().x, 24, Math.toRadians(180)),
            botStageDoorBlue = new EditablePose(botStageDoorRed.byAlliance().x, 8, Math.toRadians(0)),
            botTransitionBlue = new EditablePose(botTransitionRed.byAlliance().x, 9, Math.toRadians(0)),
            botCenterBackdropBlue = new EditablePose(botCenterBackdropRed.byAlliance().x, 35, Math.toRadians(180)),
            botLeftBackdropBlue = new EditablePose(botLeftBackdropRed.byAlliance().x, 41, Math.toRadians(180)),
            botRightBackdropBlue = new EditablePose(botRightBackdropRed.byAlliance().x, 29, Math.toRadians(180)),
            botParkingRightBlue = new EditablePose(botParkingRightRed.byAlliance().x, 10, Math.toRadians(-180)),
            botParkingLeftBlue = new EditablePose(botParkingLeftRed.byAlliance().x, 60, Math.toRadians(-180)),
            botLeftSpikeMovementBlue = new EditablePose(botLeftSpikeMovementRed.byAlliance().x, 45, Math.toRadians(-180));
    public static Pose2d
            mainSpikeBlue = null,
            mainSpikeRed = null;

    private static Pose2d
            // Global
            start,
            mainSpikeMark = null,
            botWhitePixelScoring = null,
            yellowPixel = null,
            // Local to bottom
            botLeftSpikeMovement,
            botWhitePixels,
            botStageDoor;

    private static Vector2d
            // Global
            parking,
            // Local to bottom
            botTransition;

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

        // Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (keyPressed(1, DPAD_RIGHT)) isTop = true;
            if (keyPressed(1, DPAD_LEFT)) isTop = false;
            if (keyPressed(1, B)) isRed = true;
            if (keyPressed(1, X)) isRed = false;
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + " " + (isTop ? "RIGHT" : "LEFT"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
        mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isTop ? "RIGHT" : "LEFT"));
        mTelemetry.update();

        waitForStart();

        robot.drivetrain.followTrajectorySequenceAsync(isTop ? getTopTrajectory() : getBottomTrajectory());

        while (opModeIsActive()) {
            robot.readSensors();

            robot.drivetrain.update();
            robot.run();

            robot.printTelemetry();
            mTelemetry.update();
        }
    }

    private TrajectorySequence getTopTrajectory() {
        double direction = toRadians(0);

        Pose2d startPoseBlue = MainAuton.topStartBlue.toPose2d();
        Pose2d startPoseRed = MainAuton.topStartRed.byBoth().toPose2d();
        Pose2d centerSpikeBlue = MainAuton.topCenterSpikeBlue.toPose2d();
        Pose2d centerSpikeRed = MainAuton.topCenterSpikeRed.byBoth().toPose2d();
        Pose2d leftSpikeBlue = MainAuton.topLeftSpikeBlue.toPose2d();
        Pose2d leftSpikeRed = MainAuton.topLeftSpikeRed.byBoth().toPose2d();
        Pose2d rightSpikeBlue = MainAuton.topRightSpikeBlue.toPose2d();
        Pose2d rightSpikeRed = MainAuton.topRightSpikeRed.byBoth().toPose2d();
        Pose2d blueBackboard = MainAuton.topBlueBackboard.toPose2d();
        Pose2d redBackboard = MainAuton.topRedBackboard.byBoth().toPose2d();
        Pose2d redParkingLeft = MainAuton.topRedParkingLeft.byBoth().toPose2d();
        Pose2d redParkingRight = MainAuton.topRedParkingRight.byBoth().toPose2d();
        Pose2d blueParkingLeft = MainAuton.topBlueParkingLeft.toPose2d();
        Pose2d blueParkingRight = MainAuton.topBlueParkingRight.toPose2d();

        switch (propPlacement) {
            case 0:
                mainSpikeBlue = leftSpikeBlue;
                mainSpikeRed = leftSpikeRed;
                direction = (isRed ? toRadians(135) : toRadians(-135));
                break;
            case 1:
                mainSpikeBlue = centerSpikeBlue;
                mainSpikeRed = centerSpikeRed;
                direction = (isRed ? FORWARD : BACKWARD);
                break;
            case 2:
                mainSpikeBlue = rightSpikeBlue;
                mainSpikeRed = rightSpikeRed;
                direction = (isRed ? (LEFT - toRadians(135)) : (LEFT - toRadians(-135)));
                break;
        }

        robot.drivetrain.setPoseEstimate(isRed ? startPoseRed : startPoseBlue);

        TrajectorySequence trajTop = robot.drivetrain
                .trajectorySequenceBuilder(isRed ? startPoseRed : startPoseBlue)
                .splineTo(isRed ? mainSpikeRed.vec() : mainSpikeBlue.vec(), direction)
                /*
                   Starts outtake 0.5 seconds after prev. action, then waits 0.25 seconds before stopping the outtake
                   Then stops after 1 second
                 */
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.intake.set(0.35))
                .addTemporalMarker(0.5 + OUTTAKE_WAIT_TIME, () -> robot.intake.set(0))
                .strafeRight(propPlacement == 2 ? (isRed ? 8 : -8) : 0.0001)
                .turn(propPlacement == 2 ? (isRed ? (RIGHT - toRadians(35)) : (RIGHT + toRadians(35))) : 0)
                .lineToSplineHeading(isRed ? redBackboard : blueBackboard)
                /*
                    Do april tag stuff here because now we can scan
                 */
                .turn(LEFT)
                /*
                 Starts the lift by updating target to row 0, then executes commands to do so (within timing)
                 It will activate flap to open, releasing the pixels
                 After doing that, it'll retract back to target row -1
                */
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.lift.setTargetRow(0);
                    robot.lift.updateTarget();
                })
                .addTemporalMarker(0.5 + OPEN_FLAP_WAIT_TIME, () -> robot.arm.setFlap(false))
                .addTemporalMarker((0.5 + OPEN_FLAP_WAIT_TIME) + SCORING_WAIT_TIME, () -> robot.arm.setArm(false))
                .lineToSplineHeading(isRed ? (isParkedLeft ? redParkingLeft : redParkingRight) : (isParkedLeft ? blueParkingLeft : blueParkingRight))
                .build();

        return trajTop;
    }

    private TrajectorySequence getBottomTrajectory() {
        // Red
        Pose2d botStartRed = MainAuton.botStartPoseRed.byAlliance().toPose2d();
        Pose2d botCenterSpikeRed = MainAuton.botCenterSpikeRed.byAlliance().toPose2d();
        Pose2d botLeftSpikeRed = MainAuton.botLeftSpikeRed.byAlliance().toPose2d();
        Pose2d botRightSpikeRed = MainAuton.botRightSpikeRed.byAlliance().toPose2d();
        Pose2d botWhitePixelRed = MainAuton.botWhitePixelRed.byAlliance().toPose2d();
        Pose2d botStageDoorRed = MainAuton.botStageDoorRed.byAlliance().toPose2d();
        Pose2d botTransitionRed = MainAuton.botTransitionRed.byAlliance().toPose2d();
        Pose2d botCenterBackdropRed = MainAuton.botCenterBackdropRed.byAlliance().toPose2d();
        Pose2d botLeftBackdropRed = MainAuton.botLeftBackdropRed.byAlliance().toPose2d();
        Pose2d botRightBackdropRed = MainAuton.botRightBackdropRed.byAlliance().toPose2d();
        Pose2d botParkingRightRed = MainAuton.botParkingRightRed.byAlliance().toPose2d();
        Pose2d botParkingLeftRed = MainAuton.botParkingLeftRed.byAlliance().toPose2d();
        Pose2d botLeftSpikeMovementRed = MainAuton.botLeftSpikeMovementRed.byAlliance().toPose2d();

        // Blue
        Pose2d botStartBlue = MainAuton.botStartBlue.bySide().toPose2d();
        Pose2d botCenterSpikeBlue = MainAuton.botCenterSpikeBlue.toPose2d();
        Pose2d botLeftSpikeBlue = MainAuton.botLeftSpikeBlue.toPose2d();
        Pose2d botRightSpikeBlue = MainAuton.botRightSpikeBlue.toPose2d();
        Pose2d botWhitePixelBlue = MainAuton.botWhitePixelBlue.toPose2d();
        Pose2d botStageDoorBlue = MainAuton.botStageDoorBlue.toPose2d();
        Pose2d botTransitionBlue = MainAuton.botTransitionBlue.toPose2d();
        Pose2d botCenterBackdropBlue = MainAuton.botCenterBackdropBlue.toPose2d();
        Pose2d botLeftBackdropBlue = MainAuton.botLeftBackdropBlue.toPose2d();
        Pose2d botRightBackdropBlue = MainAuton.botRightBackdropBlue.toPose2d();
        Pose2d botParkingRightBlue = MainAuton.botParkingRightBlue.toPose2d();
        Pose2d botParkingLeftBlue = MainAuton.botParkingLeftBlue.toPose2d();
        Pose2d botLeftSpikeMovementBlue = MainAuton.botLeftSpikeMovementBlue.toPose2d();

        switch (propPlacement) {
            case 0:
                mainSpikeMark = isRed ? botLeftSpikeRed : botLeftSpikeBlue;
                yellowPixel = isRed ? botLeftBackdropRed : botLeftBackdropBlue;
                botWhitePixelScoring = isRed ? botCenterBackdropRed : botRightBackdropBlue;
                break;
            case 1:
                mainSpikeMark = isRed ? botCenterSpikeRed : botCenterSpikeBlue;
                yellowPixel = isRed ? botCenterBackdropRed : botCenterBackdropBlue;
                botWhitePixelScoring = isRed ? botLeftBackdropRed : botRightBackdropBlue;
                break;
            case 2:
                mainSpikeMark = isRed ? botRightSpikeRed : botRightSpikeBlue;
                yellowPixel = isRed ? botRightBackdropRed : botRightBackdropBlue;
                botWhitePixelScoring = isRed ? botLeftBackdropRed : botCenterBackdropBlue;
                break;
        }

        if (isRed) {
            start = botStartRed;
            botLeftSpikeMovement = botLeftSpikeMovementRed;
            botWhitePixels = botWhitePixelRed;
            botStageDoor = botStageDoorRed;
            botTransition = botTransitionRed.vec();
            parking = (isParkedLeft ? botParkingLeftRed : botParkingRightRed).vec();
        } else {
            start = botStartBlue;
            botLeftSpikeMovement = botLeftSpikeMovementBlue;
            botWhitePixels = botWhitePixelBlue;
            botStageDoor = botStageDoorBlue;
            botTransition = botTransitionBlue.vec();
            parking = (isParkedLeft ? botParkingLeftBlue : botParkingRightBlue).vec();
        }

        robot.drivetrain.setPoseEstimate(start);

        TrajectorySequenceBuilder backTrajectoryBuilder = robot.drivetrain
                .trajectorySequenceBuilder(start)
                .lineToSplineHeading(mainSpikeMark);

        if (propPlacement == 0 /* Left prop */) {
            backTrajectoryBuilder.lineToSplineHeading(botLeftSpikeMovement);
        }

        backTrajectoryBuilder
                .lineToLinearHeading(botWhitePixels)
                .strafeRight(isRed ? 4 : -4)
                .splineToSplineHeading(botStageDoor, Math.toRadians(0))
                .splineTo(botTransition, Math.toRadians(0))
                .lineToSplineHeading(botWhitePixelScoring)
                .lineToSplineHeading(yellowPixel)
                .lineTo(parking);

        return backTrajectoryBuilder.build();
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
            if (isTop != isRed) x += (X_START_BOTTOM - X_START_TOP);
            return this;
        }

        EditablePose byBoth() {
            return byAlliance().bySide();
        }

        Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }
    }
}
