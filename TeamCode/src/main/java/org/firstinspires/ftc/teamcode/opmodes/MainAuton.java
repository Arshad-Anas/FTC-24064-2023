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

    public static int propPlacement = -1;

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
    public static EditablePose
            botStartRed = new EditablePose(X_START_BOTTOM, -61.788975, FORWARD),
            botLeftSpikeRed = new EditablePose(-47, -44, FORWARD),
            botCenterSpikeRed = new EditablePose(-39, -37, FORWARD),
            botRightSpikeRed = new EditablePose(-40, -34, RIGHT),
            botLeftPixelDodgeRed = new EditablePose(-53, -44, LEFT),
            botCenterPixelDodgeRed = new EditablePose (-53, -38, LEFT),
            botRightPixelDodgeRed = new EditablePose(-34, -34, RIGHT),
            botWhitePixelRed = new EditablePose(-53,-24, LEFT),
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
        propSensor = new PropSensor(hardwareMap, isRed);

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

        waitForStart();

        double tempPlacement = -1;
        while (tempPlacement != 0 && tempPlacement != 1 && tempPlacement != 2) {
            tempPlacement = 0;
            tempPlacement += propSensor.propPosition();
            Thread.sleep(100);
            tempPlacement += propSensor.propPosition();
            Thread.sleep(100);
            tempPlacement += propSensor.propPosition();
            Thread.sleep(100);
            tempPlacement /= 3.0;
        }

        propPlacement = (int) tempPlacement;

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
                .lineToSplineHeading(isParkedMiddle ? parkingLeft : parkingRight);

        return rightTrajectoryBuilder.build();
    }

    private TrajectorySequence getLeftTrajectory() {
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
                .lineToSplineHeading(dodge.byAlliancePose2d())
                .lineToLinearHeading(botWhitePixelRed.byAlliancePose2d())
                .forward(2)
                .back(2)
                .strafeRight(isRed ? 4 : -4)
                .splineToSplineHeading(botStageDoorRed.byAlliancePose2d(), Math.toRadians(0))
                .splineTo(botTransitionRed.byAllianceVec(), Math.toRadians(0))
                .lineToSplineHeading(botWhitePixelScoring.byAlliancePose2d())
                .lineToSplineHeading(yellowPixel.byAlliancePose2d())
                .lineTo(parking.byAllianceVec());

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
