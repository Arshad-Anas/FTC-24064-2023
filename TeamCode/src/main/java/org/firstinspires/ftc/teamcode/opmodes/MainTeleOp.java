package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

import java.util.List;

@TeleOp(group = "24064 TeleOp")
public class MainTeleOp extends LinearOpMode {

    MultipleTelemetry myTelemetry;
    List<LynxModule> hubs;
    GamepadEx Gamepad1, Gamepad2;

    @Override
    public void runOpMode() throws InterruptedException {

        myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        hubs = hardwareMap.getAll(LynxModule.class);
        Gamepad1 = new GamepadEx(gamepad1);
        Gamepad2 = new GamepadEx(gamepad2);
        ButtonReader
                control1A = new ButtonReader(Gamepad1, GamepadKeys.Button.A);
        ButtonReader[] buttonReaders = {control1A};
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            for (ButtonReader buttonReader : buttonReaders) buttonReader.readValue();

            // Prints the boolean if button A on gamepad 1 is held
            myTelemetry.addData("Pressed:", control1A.isDown());

            myTelemetry.update();
        }
    }
}