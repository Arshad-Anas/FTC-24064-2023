package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;

import java.util.List;

@TeleOp(group = "21836 TeleOp")
public class MainTeleOp extends LinearOpMode {

    MultipleTelemetry myTelemetry;
    List<LynxModule> hubs;

    @Override
    public void runOpMode() throws InterruptedException {

        myTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrivetrain drivetrain = new MecanumDrivetrain(hardwareMap, 537.7, 312);
        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();
            drivetrain.run(
                    Gamepad1.getLeftX(),
                    Gamepad1.getLeftY(),
                    Gamepad1.getRightX()
            );
            myTelemetry.update();
        }
    }
}
