package org.firstinspires.ftc.teamcode.opmodes;

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

    public static int propPlacement = 1;

    public static double
            X_START_BOTTOM = -35,
            X_START_TOP = 12,
            OUTTAKE_WAIT_TIME = 0.65,
            OPENING_SLIDE_TIME = 1.25,
            OPEN_FLAP_WAIT_TIME = 0.25,
            SCORING_WAIT_TIME = 1;

    public static final double
            LEFT = toRadians(180),
            FORWARD = toRadians(90),
            RIGHT = toRadians(0),
            BACKWARD = toRadians(270);

    public static EditablePose
            topStart = new EditablePose(X_START_TOP, -61.788975, FORWARD),
            topCenterSpike = new EditablePose((X_START_TOP + 3.5), -35.5, FORWARD),
            topLeftSpike = new EditablePose(7, -41, toRadians(120)),
            topRightSpike = new EditablePose(24 - topLeftSpike.x, topLeftSpike.y, LEFT - topLeftSpike.heading),
            topBackboardBe = new EditablePose(44, -35.5, RIGHT),
            topBackboardAf = new EditablePose(50, -35.5, RIGHT),
            topParkingLeft = new EditablePose(49, -14, toRadians(165)),
            topParkingRight = new EditablePose(49, -56, toRadians(200));

    // Bottom
    private static final EditablePose
            botStart = new EditablePose(X_START_BOTTOM, -61.788975, FORWARD),
            botCenterSpike = new EditablePose(-39, -38, FORWARD),
            botLeftSpike = new EditablePose(-47, -47, FORWARD),
            botRightSpike = new EditablePose(-38, -34, RIGHT),
            botWhitePixel = new EditablePose(-58,-24, LEFT),
            botStageDoor = new EditablePose(-25, -10, RIGHT),
            botTransition = new EditablePose(25, -9, RIGHT),
            botCenterBackdrop = new EditablePose(49, -35, LEFT),
            botLeftBackdrop = new EditablePose(49, -29, LEFT),
            botRightBackdrop = new EditablePose(49, -41, LEFT),
            botParkingLeft = new EditablePose(50, -10, LEFT),
            botParkingRight = new EditablePose(50, -60, LEFT),
            botLeftPixelDodge = new EditablePose(-55, -45, LEFT),
            botCenterPixelDodge = new EditablePose(-55, -38, LEFT);
    public static Pose2d
            mainSpike = null,
            autonEndPose = null,
            prop = null,
            botWhitePixelScoring = null,
            yellowPixel = null;

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
            if (keyPressed(1, A)) isParkedLeft = true;
            if (keyPressed(1, Y)) isParkedLeft = false;
            if (keyPressed(1, DPAD_UP)) propPlacement++;
            if (keyPressed(1, DPAD_DOWN)) propPlacement--;
            mTelemetry.addLine("Selected " + (isRed ? "RED" : "BLUE") + ", " + (isTop ? "RIGHT" : "LEFT") + ", " + (isParkedLeft ? "Parked Left" : "Parked Right") + ", " + propPlacement);
            mTelemetry.addLine("Press both shoulder buttons to confirm!");
            mTelemetry.update();
        }
        mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + ", " + (isTop ? "RIGHT" : "LEFT") + ", " + (isParkedLeft ? "Parked Left" : "Parked Right") + ", " + propPlacement);
        mTelemetry.update();

        waitForStart();

        robot.drivetrain.followTrajectorySequenceAsync(isTop ? getRightTrajectory() : getLeftTrajectory());

        while (opModeIsActive()) {
            robot.readSensors();

            robot.drivetrain.update();
            robot.run();

            robot.printTelemetry();
            mTelemetry.update();
        }

        autonEndPose = robot.drivetrain.getPoseEstimate();
    }

    private TrajectorySequence getRightTrajectory() {

        Pose2d startPose = MainAuton.topStart.byAlliance().toPose2d();
        Pose2d centerSpike = MainAuton.topCenterSpike.byAlliance().toPose2d();
        Pose2d leftSpike = MainAuton.topLeftSpike.byAlliance().toPose2d();
        Pose2d rightSpike = MainAuton.topRightSpike.byAlliance().toPose2d();
        Pose2d backboardBe = MainAuton.topBackboardBe.byAlliance().toPose2d();
        Pose2d backboardAf = MainAuton.topBackboardAf.byAlliance().toPose2d();
        Pose2d parkingLeft = MainAuton.topParkingLeft.byAlliance().toPose2d();
        Pose2d parkingRight = MainAuton.topParkingRight.byAlliance().toPose2d();

        switch (propPlacement) {
            case 0:
                mainSpike = leftSpike;
                break;
            case 1:
                mainSpike = centerSpike;
                break;
            case 2:
                mainSpike = rightSpike;
                break;
        }

        robot.drivetrain.setPoseEstimate(startPose);

        TrajectorySequenceBuilder rightTrajectoryBuilder = robot.drivetrain
                .trajectorySequenceBuilder(startPose)
                .splineTo(mainSpike.vec(), mainSpike.getHeading())
                /*
                   Starts outtake 0.5 seconds after prev. action, then waits 0.25 seconds before stopping the outtake
                   Then stops after 1 second
                 */
                .addTemporalMarker(0.75, () -> robot.intake.set(1))
                .addTemporalMarker(0.5 + OUTTAKE_WAIT_TIME, () -> robot.intake.set(0))

                .waitSeconds(2.1);

        if (propPlacement == 2 && mainSpike == rightSpike) {
            rightTrajectoryBuilder
                    .strafeRight(isRed ? 8 : -8)
                    .turn(isRed ? (RIGHT - toRadians(35)) : (RIGHT + toRadians(35)));
        }

        rightTrajectoryBuilder
                .lineToSplineHeading(backboardBe)
                /*
                    Do april tag stuff here because now we can scan
                 */
                .turn(LEFT)
                .lineTo(backboardAf.vec())
                /*
                 Starts the lift by updating target to row 0, then executes commands to do so (within timing)
                 It will activate flap to open, releasing the pixels
                 After doing that, it'll retract back to target row -1
                */
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.arm.setFlap(true))
                .UNSTABLE_addTemporalMarkerOffset((0.2 + 0.2), () -> {
                    robot.lift.setTargetRow(0);
                    robot.lift.updateTarget();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2 + OPENING_SLIDE_TIME, () -> robot.arm.setArm(true))
                .UNSTABLE_addTemporalMarkerOffset((0.2 + OPEN_FLAP_WAIT_TIME) + OPENING_SLIDE_TIME, () -> robot.arm.setFlap(false))
                .UNSTABLE_addTemporalMarkerOffset((2 + OPEN_FLAP_WAIT_TIME) + OPENING_SLIDE_TIME + SCORING_WAIT_TIME, () -> robot.arm.toggleArm())

                .waitSeconds(12)

                .lineTo(backboardBe.vec())
                .lineToSplineHeading(isParkedLeft ? parkingLeft : parkingRight);

        return rightTrajectoryBuilder.build();
    }

    private TrajectorySequence getLeftTrajectory() {

        Vector2d parking;

        Pose2d start = MainAuton.botStart.byAlliance().toPose2d();
        Pose2d centerSpike = MainAuton.botCenterSpike.byAlliance().toPose2d();
        Pose2d leftSpike = MainAuton.botLeftSpike.byAlliance().toPose2d();
        Pose2d rightSpike = MainAuton.botRightSpike.byAlliance().toPose2d();
        Pose2d centerBackdrop = MainAuton.botCenterBackdrop.byAlliance().toPose2d();
        Pose2d leftBackdrop = MainAuton.botLeftBackdrop.byAlliance().toPose2d();
        Pose2d rightBackdrop = MainAuton.botRightBackdrop.byAlliance().toPose2d();
        Pose2d parkingLeft = MainAuton.botParkingLeft.byAlliance().toPose2d();
        Pose2d parkingRight = MainAuton.botParkingRight.byAlliance().toPose2d();
        Pose2d centerDodge = MainAuton.botCenterPixelDodge.byAlliance().toPose2d();
        Pose2d leftDodge = MainAuton.botLeftPixelDodge.byAlliance().toPose2d();
        Pose2d stageDoor = MainAuton.botStageDoor.byAlliance().toPose2d();
        Pose2d transition = MainAuton.botTransition.byAlliance().toPose2d();
        Pose2d whitePixel = MainAuton.botWhitePixel.byAlliance().toPose2d();

        switch (propPlacement) {
            case 0:
                prop = isRed ? leftSpike : rightSpike;
                yellowPixel = isRed ? leftBackdrop : rightBackdrop;
                botWhitePixelScoring = isRed ? centerBackdrop : leftBackdrop;
                break;
            case 1:
                prop = centerSpike;
                yellowPixel = centerBackdrop;
                botWhitePixelScoring = leftBackdrop;
                break;
            case 2:
                prop = isRed ? rightSpike : leftSpike;
                yellowPixel = isRed ? rightBackdrop : leftBackdrop;
                botWhitePixelScoring = isRed ? leftBackdrop : centerBackdrop;
                break;
        }
        parking = isParkedLeft ? parkingLeft.vec() : parkingRight.vec();

        robot.drivetrain.setPoseEstimate(start);

        TrajectorySequenceBuilder builder = robot.drivetrain.trajectorySequenceBuilder(start)
                .lineToSplineHeading(prop);

        if ((!isRed && propPlacement == 2) || (isRed && propPlacement == 0)) {
            builder.lineToSplineHeading(leftDodge);
        }
        else if ((!isRed && propPlacement == 1) || (isRed && propPlacement == 1)) {
            builder.lineToSplineHeading(centerDodge);
        }
        builder.lineToLinearHeading(whitePixel)
                .strafeRight(isRed ? 4 : -4)
                .splineToSplineHeading(stageDoor, Math.toRadians(0))
                .splineTo(transition.vec(), Math.toRadians(0))
                .lineToSplineHeading(botWhitePixelScoring)
                .lineToSplineHeading(yellowPixel)
                .lineTo(parking);

        return builder.build();
    }

    private static class EditablePose {
        public double x, y, heading;

        private EditablePose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }

        private EditablePose byAlliance() {
            if (!isRed) y *= -1;
            if (!isRed) heading *= -1;
            return this;
        }

        Pose2d toPose2d() {
            return new Pose2d(x, y, heading);
        }
    }
}
