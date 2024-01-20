package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_435;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;


@TeleOp(group = "Single mechanism test")
public final class TestLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        BulkReader bulkReader = new BulkReader(hardwareMap);

        MotorEx leader = new MotorEx(hardwareMap, "leader", RPM_435);
        MotorEx follower = new MotorEx(hardwareMap, "follower", RPM_435);

        MotorEx[] motors = new MotorEx[]{leader, follower};
        waitForStart();

        while (opModeIsActive()) {
            // Read stuff
            bulkReader.bulkRead();

            gamepadEx1.readButtons();

            for (MotorEx motor : motors) motor.set(gamepadEx1.getLeftY());
        }
    }
}
