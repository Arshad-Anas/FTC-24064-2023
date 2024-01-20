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

    public static EditablePose
            topStart = new EditablePose(X_START_TOP, -61.788975, FORWARD),
            topCenterSpike = new EditablePose((X_START_TOP + 3.5), -31.5, FORWARD),
            topLeftSpike = new EditablePose(7, -40, toRadians(135)),
            topRightSpike = new EditablePose(26.5- topLeftSpike.x, topLeftSpike.y, LEFT - topLeftSpike.heading),
            topBackboardBe = new EditablePose(44, -35.5, LEFT),
            topBackboardAfLeft = new EditablePose(51.5, -31.5, RIGHT),
            topBackboardAfCenter = new EditablePose(51.5, -35.5, RIGHT),
            topBackboardAfRight = new EditablePose(51.5, -39.5, RIGHT),
            topParkingLeft = new EditablePose(49, -10, toRadians(165)),
            topParkingRight = new EditablePose(49, -56, toRadians(200));

    // Bottom
    public static EditablePose
            botStartRed = new EditablePose(X_START_BOTTOM, -61.788975, FORWARD),
            botLeftSpikeRed = new EditablePose(-47, -44, FORWARD),
            botCenterSpikeRed = new EditablePose(-39, -37, FORWARD),
            botRightSpikeRed = new EditablePose(-33, -34, toRadians(130 - 90)),
            botLeftPixelDodgeRed = new EditablePose(-53, -44, LEFT),
            botCenterPixelDodgeRed = new EditablePose (-53, -38, LEFT),
            botRightPixelDodgeRed = new EditablePose(-50, -33, LEFT),
            botWhitePixelRed = new EditablePose(-53,-22, LEFT),
            botStageDoorRed = new EditablePose(-25, -10, RIGHT),
            botTransitionRed = new EditablePose(25, -9, RIGHT),
            botCenterBackdropRed = new EditablePose(48, -35, LEFT),
            botLeftBackdropRed = new EditablePose(48, -29, LEFT),
            botRightBackdropRed = new EditablePose(48, -41, LEFT),
            botParkingLeftRed = new EditablePose(47.5, -10, LEFT),
            botParkingRightRed = new EditablePose(47.5, -60, LEFT);

    public static Pose2d
            mainSpike = null,
            autonEndPose = null;

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
        mTelemetry.addLine("Confirmed " + (isRed ? "RED" : "BLUE") + " " + (isTop ? "TOP" : "BOTTOM") + " " + (isParkedMiddle ? "PARK MIDDLE" : "PARK CORNER"));
        mTelemetry.update();

        propSensor = new PropSensor(hardwareMap, isRed);

        waitForStart();

        propPlacement = propSensor.propPosition();

        robot.drivetrain.followTrajectorySequenceAsync(isTop ? getTopTrajectory() : getBottomTrajectory());

        while (opModeIsActive()) {
            if (!opModeIsActive()) {
                return;
            }

            robot.readSensors();

            robot.drivetrain.update();
            robot.run();
        }

        propSensor.getCamera().stopStreaming();
        propSensor.getCamera().closeCameraDevice();
    }

    private TrajectorySequence getTopTrajectory() {

        Pose2d startPose = MainAuton.topStart.byAlliance().toPose2d();
        Pose2d centerSpike = MainAuton.topCenterSpike.byAlliance().toPose2d();
        Pose2d leftSpike = MainAuton.topLeftSpike.byAlliance().toPose2d();
        Pose2d rightSpike = MainAuton.topRightSpike.byAlliance().toPose2d();
        Pose2d backboardBe = MainAuton.topBackboardBe.byAlliance().toPose2d();

        EditablePose backboardAf = null;

        Pose2d parkingLeft = MainAuton.topParkingLeft.byAlliance().toPose2d();
        Pose2d parkingRight = MainAuton.topParkingRight.byAlliance().toPose2d();

        switch (propPlacement) {
            case 0:
                mainSpike = leftSpike;
                backboardAf = topBackboardAfLeft.byAlliance();
                break;
            case 1:
                mainSpike = centerSpike;
                backboardAf = topBackboardAfCenter.byAlliance();
                break;
            case 2:
                mainSpike = rightSpike;
                backboardAf = topBackboardAfRight.byAlliance();
                break;
        }

        robot.drivetrain.setPoseEstimate(startPose);

        TrajectorySequenceBuilder rightTrajectoryBuilder = robot.drivetrain
                .trajectorySequenceBuilder(startPose)
                .splineTo(mainSpike.vec(), mainSpike.getHeading())

                .waitSeconds(1)

                .back(2);

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
                .lineTo(backboardAf.toPose2d().vec())
                /*
                 Starts the lift by updating target to row 0, then executes commands to do so (within timing)
                 It will activate flap to open, releasing the pixels
                 After doing that, it'll retract back to target row -1
                */
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> robot.arm.setFlap(true))
                .UNSTABLE_addTemporalMarkerOffset((0.2 + 0.2), () -> {
                    robot.lift.setToAutonHeight();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2 + OPENING_SLIDE_TIME, () -> robot.arm.setArm(true))
                .UNSTABLE_addTemporalMarkerOffset((0.2 + OPEN_FLAP_WAIT_TIME) + OPENING_SLIDE_TIME, () -> robot.arm.setFlap(false))
                .UNSTABLE_addTemporalMarkerOffset((2 + OPEN_FLAP_WAIT_TIME) + OPENING_SLIDE_TIME + SCORING_WAIT_TIME, () -> robot.arm.toggleArm())

                .waitSeconds(9)

                .lineTo(backboardBe.vec())
                .lineToSplineHeading(isParkedMiddle ? parkingLeft : parkingRight);

        return rightTrajectoryBuilder.build();
    }

    private TrajectorySequence getBottomTrajectory() {
        EditablePose prop, dodge, yellowPixel, botWhitePixelScoring, parking;
        prop = dodge = yellowPixel = botWhitePixelScoring = null;

        switch (propPlacement) {
            case 0:
                prop = isRed ? botLeftSpikeRed : botRightSpikeRed;
                dodge = isRed ? botLeftPixelDodgeRed : botRightPixelDodgeRed;
                yellowPixel = isRed ? botLeftBackdropRed : botRightBackdropRed;
                botWhitePixelScoring = isRed ? botCenterBackdropRed : botLeftBackdropRed;
                break;
            case 1:
                prop = botCenterSpikeRed;
                dodge = botCenterPixelDodgeRed;
                yellowPixel = botCenterBackdropRed;
                botWhitePixelScoring = botLeftBackdropRed;
                break;
            case 2:
                prop = isRed ? botRightSpikeRed : botLeftSpikeRed;
                dodge = isRed ? botRightPixelDodgeRed : botLeftPixelDodgeRed;
                yellowPixel = isRed ? botRightBackdropRed : botLeftBackdropRed;
                botWhitePixelScoring = isRed ? botLeftBackdropRed : botCenterBackdropRed;
                break;
        }
        parking = isParkedMiddle ? botParkingLeftRed : botParkingRightRed;

        Pose2d start = botStartRed.byAlliancePose2d();
        robot.drivetrain.setPoseEstimate(start);

        TrajectorySequenceBuilder builder = robot.drivetrain.trajectorySequenceBuilder(start)
                .lineToSplineHeading(prop.byAlliancePose2d())
                .back(8)
                .lineToSplineHeading(dodge.byAlliancePose2d())
                .lineToLinearHeading(botWhitePixelRed.byAlliancePose2d())
                .forward(7)
                .addTemporalMarker(() -> robot.intake.set(-1)) // Intake
                .waitSeconds(3)
                .back(7)
                .addTemporalMarker(() -> {
                    robot.intake.set(1); // Outtake
                    robot.arm.setFlap(true); // Close flap
                })
                .strafeRight(isRed ? 4 : -4)
                .addTemporalMarker(() -> robot.intake.set(0)) // Stop outtaking
                .splineToSplineHeading(botStageDoorRed.byAlliancePose2d(), Math.toRadians(0))
                .splineTo(botTransitionRed.byAllianceVec(), Math.toRadians(0))
                .addTemporalMarker(() -> robot.lift.setToAutonHeight()) // Lift extend
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> robot.arm.setArm(true)) // Extend arm
                .lineToSplineHeading(botWhitePixelScoring.byAlliancePose2d())
                .addTemporalMarker(() -> robot.arm.setFlap(false)) // Open flap
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> robot.arm.setFlap(true)) // Close flap 0.1 seconds after opening them
                .lineToSplineHeading(yellowPixel.byAlliancePose2d())
                .addTemporalMarker(() -> robot.arm.setFlap(false))
                .UNSTABLE_addTemporalMarkerOffset(1, () -> robot.arm.toggleArm()); // This will also bring the lifts down and close the flap

        return builder.build();
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

//        private EditablePose bySide() {
//            if (isTop != isRed) x += (X_START_BOTTOM - X_START_TOP);
//            return this;
//        }
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