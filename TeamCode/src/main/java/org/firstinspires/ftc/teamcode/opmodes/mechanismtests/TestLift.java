package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Arm.ANGLE_COLLECTING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Arm.ANGLE_DEPOSITING;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.centerstage.Lift;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@TeleOp(group = "Single mechanism test")
public final class TestLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        gamepadEx1 = new GamepadEx(gamepad1);
        BulkReader bulkReader = new BulkReader(hardwareMap);
        mTelemetry = new MultipleTelemetry(telemetry);

        Lift lift = new Lift(hardwareMap);

        SimpleServoPivot arm = new SimpleServoPivot(
                ANGLE_COLLECTING,
                ANGLE_DEPOSITING,
                getGoBildaServo(hardwareMap, "arm1"),
                getReversedServo(getGoBildaServo(hardwareMap, "arm2"))
        );

        waitForStart();

        while (opModeIsActive()) {
            // Read stuff
            bulkReader.bulkRead();

            gamepadEx1.readButtons();

            if (keyPressed(1, Y)) arm.toggle();

            double stick = -gamepadEx1.getRightY();
            lift.setWithStick(stick < 0 && arm.isActivated() ? 0 : stick);

            arm.run();
            lift.run();

            mTelemetry.addData("Arm is", arm.isActivated());
            lift.printNumericalTelemetry();
            mTelemetry.update();
        }
    }
}
