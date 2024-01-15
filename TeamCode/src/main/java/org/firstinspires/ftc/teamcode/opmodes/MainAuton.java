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
    public static MultipleTelemetry mTelemetry;
    public static GamepadEx gamepadEx1, gamepadEx2;

    static boolean
            isRed = false,
            isParkedLeft = true,
            isRight = true;

    public static int spikeNum = 2;

    public static double
            X_START_LEFT = -35,
            X_START_RIGHT = 12,
            OUTTAKE_WAIT_TIME = 0.25,
            SCORING_WAIT_TIME = 0.75,
            OPEN_FLAP_WAIT_TIME = 0.25;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static EditablePose
            topStartRed = new EditablePose(X_START_RIGHT, -61.788975, FORWARD),
            topStartBlue = new EditablePose(topStartRed.byAlliance().x, 61, BACKWARD),
            topCenterSpikeRed = new EditablePose((X_START_RIGHT + 3.5), -33.5, FORWARD),
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

        robot.drivetrain.followTrajectorySequenceAsync(getTopTrajectory());

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

        switch (spikeNum) {
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
                .strafeRight(spikeNum == 2 ? (isRed ? 8 : -8) : 0.0001)
                .turn(spikeNum == 2 ? (isRed ? (RIGHT - toRadians(35)) : (RIGHT + toRadians(35))) : 0)
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

        Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }
    }
}
