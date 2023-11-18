package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.List;

@Config
//@Autonomous(group = "24064 Autonomous", preselectTeleOp = "MainTeleOp")
public abstract class BaseAuton extends LinearOpMode {

    MultipleTelemetry myTelemetry;
    List<LynxModule> hubs;

    public void runOpMode(boolean isRed, boolean isRight) throws InterruptedException {

        myTelemetry = new MultipleTelemetry(telemetry);

        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        waitForStart();

        while (opModeIsActive()) {
            for (LynxModule hub : hubs) hub.clearBulkCache();

            myTelemetry.update();
        }
    }
}
