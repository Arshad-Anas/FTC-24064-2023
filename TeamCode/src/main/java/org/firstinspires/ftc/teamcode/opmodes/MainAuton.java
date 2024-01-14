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
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

@Config
@Autonomous(group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {

    static Robot robot;
    static MultipleTelemetry mTelemetry;
    static GamepadEx gamepadEx1, gamepadEx2;

    static boolean isRed = false, isTop = false, isParkedLeft = true;
    static int propPlacement = 1;

    static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static double
            X_START_BOTTOM = -35,
            X_START_TOP = 12;

    // Bottom
    public static EditablePose
            // This is red alliance
            botStartPoseRed = new EditablePose(X_START_BOTTOM, -61.788975, FORWARD),
            botCenterSpikeRed = new EditablePose(-39, -38,Math.toRadians(90)),
            botLeftSpikeRed = new EditablePose(-46.5, -47, toRadians(90)),
            botRightSpikeRed = new EditablePose(11 + botLeftSpikeRed.x, 13 + botLeftSpikeRed.y, Math.toRadians(0)),

            // The below 6 are for center spike red alliance
            botWhitePixelRed = new EditablePose(-58,-24,Math.toRadians(-180)),
            botStageDoorRed = new EditablePose(-25, -10,Math.toRadians(0)), //15 could also work here if dBRed is removed, but this require further testing
            botTransitionRed = new EditablePose(25,-9,Math.toRadians(0)),
            botCenterBackdropRed = new EditablePose(49,-35,Math.toRadians(-180)),
            botLeftBackdropRed = new EditablePose(49,-30, Math.toRadians(-180)),
            botRightBackdropRed = new EditablePose(49,-41, Math.toRadians(-180)),
            botParkingLeftRed = new EditablePose(50, -10, Math.toRadians(-180)),
            botParkingRightRed = new EditablePose(50,-60, Math.toRadians(-180)),
            botLeftSpikeMovementRed = new EditablePose(-55,-45, Math.toRadians(180)),

            // This is for blue alliance
            botStartBlue = new EditablePose(botStartPoseRed.byAlliance().x, 61.788975,BACKWARD),
            botCenterSpikeBlue = new EditablePose(botCenterSpikeRed.byAlliance().x, 38, Math.toRadians(-90)),
            botLeftSpikeBlue = new EditablePose(botLeftSpikeRed.byAlliance().x, 47, toRadians(-90)),
            botRightSpikeBlue = new EditablePose(botRightSpikeRed.byAlliance().x, 34, Math.toRadians(0)),

            // The below 6 are for center spike blue alliance
            botWhitePixelBlue = new EditablePose(botWhitePixelRed.byAlliance().x,24,Math.toRadians(180)),
            botStageDoorBlue = new EditablePose(botStageDoorRed.byAlliance().x, 8,Math.toRadians(0)),
            botTransitionBlue = new EditablePose(botTransitionRed.byAlliance().x,9,Math.toRadians(0)),
            botCenterBackdropBlue = new EditablePose(botCenterBackdropRed.byAlliance().x,35,Math.toRadians(180)),
            botLeftBackdropBlue = new EditablePose(botLeftBackdropRed.byAlliance().x, 41, Math.toRadians(180)),
            botRightBackdropBlue = new EditablePose(botRightBackdropRed.byAlliance().x, 29, Math.toRadians(180)),
            botParkingRightBlue = new EditablePose(botParkingRightRed.byAlliance().x, 10, Math.toRadians(-180)),
            botParkingLeftBlue = new EditablePose(botParkingLeftRed.byAlliance().x, 60, Math.toRadians(-180)),
            botLeftSpikeMovementBlue = new EditablePose(botLeftSpikeMovementRed.byAlliance().x, 45, Math.toRadians(-180));


    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry);

        // Initialize gamepad (ONLY FOR INIT, DON'T CALL DURING WHILE LOOP)
        gamepadEx1 = new GamepadEx(gamepad1);

        robot = new Robot(hardwareMap);

        // Get gamepad 1 button input and save "right" and "red" booleans for autonomous configuration:
        boolean right = true, red = true;
        while (opModeInInit() && !(gamepadEx1.isDown(RIGHT_BUMPER) && gamepadEx1.isDown(LEFT_BUMPER))) {
            gamepadEx1.readButtons();
            if (keyPressed(1, DPAD_RIGHT)) right = true;
            if (keyPressed(1, DPAD_LEFT)) right = false;
            if (keyPressed(1, B)) red = true;
            if (keyPressed(1, X)) red = false;
            mTelemetry.addLine("Selected " + (red ? "RED" : "BLUE") + " " + (right ? "RIGHT" : "LEFT"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
        mTelemetry.addLine("Confirmed " + (red ? "RED" : "BLUE") + " " + (right ? "RIGHT" : "LEFT"));
        mTelemetry.update();

        // Red bottom
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

        // Blue bottom
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

        // Global
        Pose2d mainSpikeMark = null;
        Pose2d yellowPixel = null;
        Vector2d parking;

        // Local to bottom
        Pose2d botWhitePixels;
        Pose2d botStageDoor;
        Vector2d botTransition;

        switch (propPlacement) {
            case 0:
                mainSpikeMark = isRed ? botLeftSpikeRed : botLeftSpikeBlue;
                yellowPixel = isRed ? botLeftBackdropRed : botLeftBackdropBlue;
            case 1:
                mainSpikeMark = isRed ? botCenterSpikeRed : botCenterSpikeBlue;
                yellowPixel = isRed ? botCenterBackdropRed : botCenterBackdropBlue;
            case 2:
                mainSpikeMark = isRed ? botRightSpikeRed : botRightSpikeBlue;
                yellowPixel = isRed ? botRightBackdropRed : botRightBackdropBlue;
        }

        if (isRed) {
            botWhitePixels = botWhitePixelRed;
            botStageDoor = botStageDoorRed;
            botTransition = botTransitionRed.vec();
            parking = (isParkedLeft ? botParkingLeftRed : botParkingRightRed).vec();
        } else {
            botWhitePixels = botWhitePixelBlue;
            botStageDoor = botStageDoorBlue;
            botTransition = botTransitionBlue.vec();
            parking = (isParkedLeft ? botParkingLeftBlue : botParkingRightBlue).vec();
        }

        TrajectorySequence trajBack = robot.drivetrain.trajectorySequenceBuilder(isRed ? botStartRed : botStartBlue)
                                .lineToSplineHeading(mainSpikeMark)
                                .lineToSplineHeading(isRed ? botLeftSpikeMovementRed : botLeftSpikeMovementBlue) // TODO DO TERNARY IF LEFT SPIKE
                                .lineToLinearHeading(botWhitePixels)
                                .strafeRight(isRed ? 4 : -4)
                                .splineToSplineHeading(botStageDoor, Math.toRadians(0))
                                .splineTo(botTransition, Math.toRadians(0))
                                .lineToSplineHeading(yellowPixel)
                                .lineTo(parking)
                                .build();

        waitForStart();

        while (opModeIsActive()) {

            robot.readSensors();

            robot.drivetrain.update();
            robot.run();

            mTelemetry.update();
        }

        robot.interrupt();
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

        private EditablePose byBoth() {
            return byAlliance().bySide();
        }

        private Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }
    }
}
