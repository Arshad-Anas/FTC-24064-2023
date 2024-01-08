package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;

import static java.lang.Math.PI;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Arm;
import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

@Config
@Autonomous(group = "24064 Main", preselectTeleOp = "MainTeleOp")
public final class MainAuton extends LinearOpMode {
    static Robot robot;
    public static MultipleTelemetry mTelemetry;
    public static GamepadEx gamepadEx1, gamepadEx2;

    static boolean isRed = false, isParkedLeft = true, isRightCenterSpike = false;

    static boolean isRight = true;

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static int spikeNum = 2;

    public static EditablePose
            startPoseRed = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            startPoseBlue = new EditablePose(startPoseRed.byAlliance().toPose2d().vec().getX(), 61, BACKWARD),
            centerSpikeRed = new EditablePose((X_START_RIGHT + 3.5), -33.5, FORWARD),
            centerSpikeBlue = new EditablePose(centerSpikeRed.byAlliance().toPose2d().vec().getX(), 33.5, BACKWARD),
            leftSpikeRed = new EditablePose(7, -41, toRadians(120)),
            leftSpikeBlue = new EditablePose(leftSpikeRed.byAlliance().toPose2d().vec().getX(), 41, toRadians(-120)),
            rightSpikeRed = new EditablePose(24 - leftSpikeRed.x, leftSpikeRed.y, LEFT - leftSpikeRed.heading),
            rightSpikeBlue = new EditablePose(rightSpikeRed.byAlliance().toPose2d().vec().getX(), 41, LEFT + leftSpikeRed.heading),
            redBackboard = new EditablePose(48, -34, RIGHT),
            blueBackboard = new EditablePose(redBackboard.byAlliance().toPose2d().vec().getX(), 34, RIGHT),
            redParkingLeft = new EditablePose(52, -14, toRadians(165)),
            redParkingRight = new EditablePose(51, -54, toRadians(200)),
            blueParkingLeft = new EditablePose(redParkingLeft.toPose2d().vec().getX(), 60, toRadians(165)),
            blueParkingRight = new EditablePose(redParkingRight.toPose2d().vec().getX(), 14, toRadians(200));

    public static Pose2d
            mainSpikeBlue = null,
            mainSpikeRed = null;

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
            if (keyPressed(1, DPAD_RIGHT)) isRight = true;
            if (keyPressed(1, DPAD_LEFT)) isRight = false;
            if (keyPressed(1, B)) isRed = true;
            if (keyPressed(1, X)) isRed = false;
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
        mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isRight ? "RIGHT" : "LEFT"));
        mTelemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            robot.readSensors();

            robot.drivetrain.update();
            robot.run();

            robot.printTelemetry();
            mTelemetry.update();

            double side = isRed ? 1 : -1;
            double direction = toRadians(0);

            double
                    OUTTAKE_WAIT_TIME = 0.25,
                    SCORING_WAIT_TIME = 0.75,
                    OPEN_FLAP_WAIT_TIME = 0.25;

            Pose2d startPoseBlue = MainAuton.startPoseBlue.toPose2d();
            Pose2d startPoseRed = MainAuton.startPoseRed.byBoth().toPose2d();
            Pose2d centerSpikeBlue = MainAuton.centerSpikeBlue.toPose2d();
            Pose2d centerSpikeRed = MainAuton.centerSpikeRed.byBoth().toPose2d();
            Pose2d leftSpikeBlue = MainAuton.leftSpikeBlue.toPose2d();
            Pose2d leftSpikeRed = MainAuton.leftSpikeRed.byBoth().toPose2d();
            Pose2d rightSpikeBlue = MainAuton.rightSpikeBlue.toPose2d();
            Pose2d rightSpikeRed = MainAuton.rightSpikeRed.byBoth().toPose2d();
            Pose2d blueBackboard = MainAuton.blueBackboard.toPose2d();
            Pose2d redBackboard = MainAuton.redBackboard.byBoth().toPose2d();
            Pose2d redParkingLeft = MainAuton.redParkingLeft.byBoth().toPose2d();
            Pose2d redParkingRight = MainAuton.redParkingRight.byBoth().toPose2d();
            Pose2d blueParkingLeft = MainAuton.blueParkingLeft.toPose2d();
            Pose2d blueParkingRight = MainAuton.blueParkingRight.toPose2d();

            switch (spikeNum) {
                case 0: {
                    mainSpikeBlue = leftSpikeBlue;
                    mainSpikeRed = leftSpikeRed;
                    break;
                }

                case 1: {
                    mainSpikeBlue = centerSpikeBlue;
                    mainSpikeRed = centerSpikeRed;
                    break;
                }

                case 2: {
                    mainSpikeBlue = rightSpikeBlue;
                    mainSpikeRed = rightSpikeRed;
                    isRightCenterSpike = true;
                    break;
                }
            }

            switch (spikeNum) {
                case 0: {
                    direction = (isRed ? toRadians(135) : toRadians(-135));
                    break;
                }

                case 1: {
                    direction = (isRed ? FORWARD : BACKWARD);
                    break;
                }

                case 2: {
                    direction = (isRed ? (LEFT - toRadians(135)) : (LEFT - toRadians(-135)));
                    break;
                }
            }

            robot.drivetrain.setPoseEstimate(isRed ? startPoseRed : startPoseBlue);

            double finalDirection = direction;
            TrajectorySequence trajSequenceTop = robot.drivetrain.trajectorySequenceBuilder(isRed ? startPoseRed : startPoseBlue)

                    .splineTo(isRed ? mainSpikeRed.vec() : mainSpikeBlue.vec(), finalDirection)
                    /*
                       Starts outtake 0.5 seconds after prev. action, then waits 0.25 seconds before stopping the outtake
                       Then stops after 1 second
                     */
                    .UNSTABLE_addTemporalMarkerOffset(0.5, () -> robot.intake.set(0.35))
                    .addTemporalMarker(0.5 + OUTTAKE_WAIT_TIME, () -> robot.intake.set(0))
                    .strafeRight(isRightCenterSpike ? (isRed ? 8 : -8) : 0.0001)
                    .turn(isRightCenterSpike ? RIGHT - toRadians(35) : 0)
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
                     .addTemporalMarker(0.5 + OPEN_FLAP_WAIT_TIME, () -> robot.arm.setFlap(true))
                     .addTemporalMarker((0.5 + OPEN_FLAP_WAIT_TIME) + SCORING_WAIT_TIME, () -> robot.arm.setArm(true))
                    .lineToSplineHeading(isRed ? (isParkedLeft ? redParkingLeft : redParkingRight) : (isParkedLeft ? blueParkingLeft : blueParkingRight))
                    .build();
            }
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

        EditablePose byBoth() {
            return byAlliance().bySide();
        }

        private EditablePose flipBySide() {
            boolean isRight = MainAuton.isRight == isRed;
            if (!isRight) heading = PI - heading;
            if (!isRight) x = (X_START_LEFT + X_START_RIGHT) / 2 - x;
            return this;
        }

        Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }
    }
}
