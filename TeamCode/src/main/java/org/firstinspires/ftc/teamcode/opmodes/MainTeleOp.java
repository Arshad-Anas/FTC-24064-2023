package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.X;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx2;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
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
        // Initialize multiple telemetry outputs:
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // The gamepads are set to GamepadEx objects
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        // Instantiated the robot class
        robot = new Robot(hardwareMap);

        waitForStart();

        // Control loop:
        while (opModeIsActive()) {
            robot.readSensors();
            // Read sensors + gamepads:
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // Changes the level of the lift, which is changed by the d-pad
             if (keyPressed(2, GamepadKeys.Button.DPAD_UP)) robot.lift.increment();
             if (keyPressed(2, GamepadKeys.Button.DPAD_DOWN)) robot.lift.decrement();
             if (keyPressed(2, GamepadKeys.Button.A)) robot.lift.updateTarget();

            // The intake's motor power is set by the tuning of the triggers on the gamepad
            robot.intake.setMotorPower(gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER));

            robot.run();

            // Field-centric drive dt with control stick inputs:
            robot.drivetrain.run(
                    -gamepadEx1.getLeftX(),
                    -gamepadEx1.getLeftY(),
                    -gamepadEx1.getRightX()
            );

            robot.printTelemetry(mTelemetry);
            mTelemetry.update();
        }
        robot.interrupt();
    }
}