package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Robot;

import java.util.List;

@TeleOp(group = "24064 TeleOp")
public class MainTeleOp extends LinearOpMode {

    Robot robot;

    MultipleTelemetry myTelemetry;

    List<LynxModule> hubs;

    GamepadEx gamepad1, gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize multiple telemetry outputs:
        myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize internal hub representations:
        // Switch hubs to manually reset sensor inputs when we tell it to:
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        gamepad1 = new GamepadEx(super.gamepad1);
        gamepad2 = new GamepadEx(super.gamepad2);

        robot = new Robot(hardwareMap);

        waitForStart();

        robot.start();

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            for (LynxModule hub : hubs) hub.clearBulkCache();
            // Read sensors + gamepads:
            gamepad1.readButtons();
            gamepad2.readButtons();

            if (gamepad1.wasJustPressed(DPAD_UP)) robot.lift.increment();
            if (gamepad1.wasJustPressed(DPAD_DOWN)) robot.lift.decrement();

            robot.intake.setMotorPower(gamepad1.getTrigger(RIGHT_TRIGGER) - gamepad1.getTrigger(LEFT_TRIGGER));

            robot.run();

            // Field-centric drive dt with control stick inputs:
            robot.drivetrain.run(
                    -gamepad1.getLeftX(),
                    -gamepad1.getLeftY(),
                    -gamepad1.getRightX()
            );

            robot.printTelemetry(myTelemetry);
            myTelemetry.update();
        }
        robot.interrupt();
    }
}