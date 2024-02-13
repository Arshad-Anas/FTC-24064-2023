package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.mTelemetry;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.vision.AprilTagLocalization;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;

@TeleOp(group = "Single mechanism test")
public final class TestAprilTagLocalization extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        mTelemetry = new MultipleTelemetry(telemetry);

        BulkReader bulkReader = new BulkReader(hardwareMap);
        AprilTagLocalization localization = new AprilTagLocalization(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            bulkReader.bulkRead();

            mTelemetry.addData("Pose estimate (inches)", localization.getPoseEstimate());
            mTelemetry.update();

            sleep(20);
        }
    }
}
