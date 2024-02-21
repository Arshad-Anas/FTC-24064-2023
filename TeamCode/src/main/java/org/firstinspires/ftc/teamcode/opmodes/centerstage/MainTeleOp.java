package org.firstinspires.ftc.teamcode.opmodes.centerstage;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.BACKWARD;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.FORWARD;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.autonEndPose;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.isRed;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.robot;
import static java.lang.Math.atan2;
import static java.lang.Math.hypot;
import static java.lang.Math.max;
import static java.lang.Math.pow;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Robot;

@TeleOp(group = "24064 Main")
public final class MainTeleOp extends LinearOpMode {

    /**
     * OpMode that is shown in driver hub; Calls all the classes and objs
     */
    @Override
    public void runOpMode() throws InterruptedException {
        boolean isHanging = false;

        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        robot = new Robot(hardwareMap);

        if (autonEndPose != null)
            robot.drivetrain.setCurrentHeading(autonEndPose.getHeading() - (isRed ? FORWARD : BACKWARD));

        robot.purplePixel.setActivated(true);

        waitForStart();

        while (opModeIsActive()) {
            // Read sensors + gamepads:
            robot.readSensors();
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // Gamepad 1
            // Change the heading of the drivetrain in field-centric mode
            double x = gamepadEx1.getRightX();
            if (gamepadEx1.isDown(LEFT_BUMPER)) {
                double y = gamepadEx1.getRightY();
                if (hypot(x, y) >= 0.8) robot.drivetrain.setCurrentHeading(atan2(y, x));
                x = 0;
            }

            if (gamepadEx1.isDown(RIGHT_BUMPER)) {
                robot.drivetrain.run(
                        gamepadEx1.getLeftX() * 0.2,
                        gamepadEx1.getLeftY() * 0.2,
                        x * 0.2
                );
            } else {
                robot.drivetrain.run(
                        gamepadEx1.getLeftX(),
                        gamepadEx1.getLeftY(),
                        x
                );
            }
            if (keyPressed(1, X) && robot.launcherClamp.isActivated()) robot.launcher.toggle();
            if (keyPressed(1, A)) robot.launcherClamp.toggle();
            if (keyPressed(1, DPAD_UP)) isHanging = !isHanging;
//            if (keyPressed(1, B)) robot.deployableRoller.toggle();

            // Gamepad 2
            double stick = pow(gamepadEx2.getLeftY(), 3);
            if (stick != 0) robot.lift.setWithStick(stick);
            if (keyPressed(2, B)) robot.arm.toggleFlap();
            if (robot.lift.getSetPoint() >= 0) {
                if (keyPressed(2, Y)) {
                    robot.arm.toggleArm();
                    robot.wrist.toggle();
                }
            }

            // Shared
            // The intake power takes precedent to the first player
            double trigger1 = gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER);
            double trigger2 = gamepadEx2.getTrigger(RIGHT_TRIGGER) - gamepadEx2.getTrigger(LEFT_TRIGGER);
            double intake = trigger1 != 0 ? trigger1 : trigger2;
            if (!isHanging) robot.rollers.intake(intake);
            robot.rollers.setDeployableWithTrigger(intake);

            robot.drivetrain.update();

            if (!isHanging) robot.run();
            else robot.hang(trigger1);

            robot.printTelemetry();
            mTelemetry.update();
        }
    }
}