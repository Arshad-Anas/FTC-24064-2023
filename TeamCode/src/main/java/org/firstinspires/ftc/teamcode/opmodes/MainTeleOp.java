package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.MecanumDrivetrain;

import java.util.List;

@TeleOp(group = "24064 TeleOp")
public class MainTeleOp extends LinearOpMode {

    MultipleTelemetry myTelemetry;
    List<LynxModule> hubs;
    GamepadEx Gamepad1, Gamepad2;
    MecanumDrivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize multiple telemetry outputs:
        myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize internal hub representations:
        // Switch hubs to manually reset sensor inputs when we tell it to:
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);

        drivetrain = new MecanumDrivetrain(hardwareMap);

        waitForStart();
        drivetrain.imu.start();
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Control loop:
        while (opModeIsActive()) {
            // Manually clear old sensor data from the last loop:
            for (LynxModule hub : hubs) hub.clearBulkCache();
            // Read sensors + gamepads:
            Gamepad1.readButtons();
            Gamepad2.readButtons();

            // Field-centric drive dt with control stick inputs:
            drivetrain.run(
                    -Gamepad1.getLeftX(),
                    -Gamepad1.getLeftY(),
                    -Gamepad1.getRightX()
            );

            intake.setPower(Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));

            // Push telemetry data to multiple outputs (set earlier):
            myTelemetry.addData("Pressed:", Gamepad1.isDown(A));
            myTelemetry.addData("Intake power: ", Gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            myTelemetry.update();
        }
        drivetrain.imu.interrupt();
    }
}