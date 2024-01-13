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

    static boolean isRed = false, isRight = false, isParkedLeft = true;

    static boolean keyPressed(int gamepad, GamepadKeys.Button button) {
        return (gamepad == 2 ? gamepadEx2 : gamepadEx1).wasJustPressed(button);
    }

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12;

    public static EditablePose
            // this is red alliance
            botStartPoseRed = new EditablePose(X_START_LEFT, -61.788975, FORWARD),
            botCenterSpikeRed = new EditablePose(-39, -38,Math.toRadians(90)),
            botLeftSpikeRed = new EditablePose(-46.5, -47, toRadians(90)),
            botRightSpikeRed = new EditablePose(11 + botLeftSpikeRed.x, 13 + botLeftSpikeRed.y, Math.toRadians(0)),

    // the below 6 are for center spike red alliance
    //  firstSpikeRed = new EditablePose(-49,-24,Math.toRadians(0)),
    botWhitePixelRed = new EditablePose(-58,-24,Math.toRadians(-180)),
            botStageDoorRed = new EditablePose(-25, -10,Math.toRadians(0)), //15 could also work here if dBRed is removed, but this require further testing
            botRedTransition = new EditablePose(25,-9,Math.toRadians(0)),
            botCenterBackdropRed = new EditablePose(49,-35,Math.toRadians(-180)),
            botLeftBackdropRed = new EditablePose(49,-30, Math.toRadians(-180)),
            botRightBackdropRed = new EditablePose(49,-41, Math.toRadians(-180)),
            botParkingLeftRed = new EditablePose(50, -10, Math.toRadians(-180)),
            botParkingRightRed = new EditablePose(50,-60, Math.toRadians(-180)),
            botLeftSpikeMovementRed = new EditablePose(-55,-45, Math.toRadians(180)),

    // This is for blue alliance
    botStartBlue = new EditablePose(botStartPoseRed.byAlliance().toPose2d().vec().getX(), 61.788975,BACKWARD),
            botCenterSpikeBlue = new EditablePose(botCenterSpikeRed.byAlliance().toPose2d().vec().getX(), 38, Math.toRadians(-90)),
            botLeftSpikeBlue = new EditablePose(botLeftSpikeRed.byAlliance().toPose2d().vec().getX(), 47, toRadians(-90)),
            botRightSpikeBlue = new EditablePose(botRightSpikeRed.byAlliance().toPose2d().vec().getX(), 34, Math.toRadians(0)),

    // the below 6 are for center spike blue alliance
    //   firstSpikeBlue = new EditablePose(firstSpikeRed.byAlliance().toPose2d().vec().getX(),24,Math.toRadians(0)),
    botWhitePixelBlue = new EditablePose(botWhitePixelRed.byAlliance().toPose2d().vec().getX(),24,Math.toRadians(180)),
            botStageDoorBlue = new EditablePose(botStageDoorRed.byAlliance().toPose2d().vec().getX(), 8,Math.toRadians(0)),
            botTransitionBlue = new EditablePose(botRedTransition.byAlliance().toPose2d().vec().getX(),9,Math.toRadians(0)),
            botCenterBackdropBlue = new EditablePose(botCenterBackdropRed.byAlliance().toPose2d().vec().getX(),35,Math.toRadians(180)),
            botLeftBackdropBlue = new EditablePose(botLeftBackdropRed.byAlliance().toPose2d().vec().getX(), 41, Math.toRadians(180)),
            botRightBackdropBlue = new EditablePose(botRightBackdropRed.byAlliance().toPose2d().vec().getX(), 29, Math.toRadians(180)),
            botParkingRightBlue = new EditablePose(botParkingRightRed.byAlliance().toPose2d().vec().getX(), 10, Math.toRadians(-180)),
            botParkingLeftBlue = new EditablePose(botParkingLeftRed.byAlliance().toPose2d().vec().getX(), 60, Math.toRadians(-180)),
            botLeftSpikeMovementBlue = new EditablePose(botLeftSpikeMovementRed.byAlliance().toPose2d().vec().getX(), 45, Math.toRadians(-180));


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

        Pose2d botStartRed = MainAuton.botStartPoseRed.byAlliance().toPose2d();
        Pose2d botCenterSpikeRed = MainAuton.botCenterSpikeRed.byAlliance().toPose2d();
        Pose2d botLeftSpikeRed = MainAuton.botLeftSpikeRed.byAlliance().toPose2d();
        Pose2d botRightSpikeRed = MainAuton.botRightSpikeRed.byAlliance().toPose2d();
        Pose2d botWhitePixelRed = MainAuton.botWhitePixelRed.byAlliance().toPose2d();
        Pose2d botStageDoorRed = MainAuton.botStageDoorRed.byAlliance().toPose2d();
        Pose2d botRedTransition = MainAuton.botRedTransition.byAlliance().toPose2d();
        Pose2d botCenterBackdropRed = MainAuton.botCenterBackdropRed.byAlliance().toPose2d();
        Pose2d botLeftBackdropRed = MainAuton.botLeftBackdropRed.byAlliance().toPose2d();
        Pose2d botRightBackdropRed = MainAuton.botRightBackdropRed.byAlliance().toPose2d();
        Pose2d botParkingRightRed = MainAuton.botParkingRightRed.byAlliance().toPose2d();
        Pose2d botParkingLeftRed = MainAuton.botParkingLeftRed.byAlliance().toPose2d();
        Pose2d botLeftSpikeMovementRed = MainAuton.botLeftSpikeMovementRed.byAlliance().toPose2d();

        Pose2d botStartBlue = MainAuton.botStartBlue.bySide().toPose2d();
        Pose2d botCenterSpikeBlue = MainAuton.botCenterSpikeBlue.toPose2d();
        Pose2d botLeftSpikeBlue = MainAuton.botLeftSpikeBlue.toPose2d();
        Pose2d botRightSpikeBlue = MainAuton.botRightSpikeBlue.toPose2d();
        Pose2d botWhitePixelBlue = MainAuton.botWhitePixelBlue.toPose2d();
        Pose2d botStageDoorBlue = MainAuton.botStageDoorBlue.toPose2d();
        Pose2d botBlueTransition = MainAuton.botTransitionBlue.toPose2d();
        Pose2d botCenterBackdropBlue = MainAuton.botCenterBackdropBlue.toPose2d();
        Pose2d botLeftBackdropBlue = MainAuton.botLeftBackdropBlue.toPose2d();
        Pose2d botRightBackdropBlue = MainAuton.botRightBackdropBlue.toPose2d();
        Pose2d botParkingRightBlue = MainAuton.botParkingRightBlue.toPose2d();
        Pose2d botParkingLeftBlue = MainAuton.botParkingLeftBlue.toPose2d();
        Pose2d botLeftSpikeMovementBlue = MainAuton.botLeftSpikeMovementBlue.toPose2d();

        TrajectorySequence trajBack = robot.drivetrain.trajectorySequenceBuilder(isRed ? botStartRed : botStartBlue)
                                .lineToSplineHeading(isRed ? botLeftSpikeRed : botLeftSpikeBlue)
                                //    .addDisplacementMarker((){})
                                .lineToSplineHeading(isRed ? botLeftSpikeMovementRed : botLeftSpikeMovementBlue) //DO TURNARY IF LEFT SPIKE
                                .lineToLinearHeading(isRed ? botWhitePixelRed : botWhitePixelBlue)
                                .strafeRight(isRed ? 4 : -4)
                                .splineToSplineHeading(isRed ? botStageDoorRed : botStageDoorBlue, Math.toRadians(0))
                                .splineTo(isRed ? botRedTransition.vec() : botBlueTransition.vec(), Math.toRadians(0))
                                .lineToSplineHeading(isRed ? botLeftBackdropRed : botLeftBackdropBlue )
                                .lineTo(isRed ? (isParkedLeft ? botParkingLeftRed.vec() : botParkingRightRed.vec()) : (isParkedLeft ? botParkingLeftBlue.vec() : botParkingRightBlue.vec()))
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
            if (isRight != isRed) x += (X_START_LEFT - X_START_RIGHT);
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
