package org.firstinspires.ftc.teamcode.opmodes.mechanismtests;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.Y;
import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_435;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.gamepadEx1;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.keyPressed;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Arm.ANGLE_COLLECTING;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Arm.ANGLE_DEPOSITING;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;


@TeleOp(group = "Single mechanism test")
public final class TestLift extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize gamepads:
        gamepadEx1 = new GamepadEx(gamepad1);
        BulkReader bulkReader = new BulkReader(hardwareMap);

        MotorEx leader = new MotorEx(hardwareMap, "leader", RPM_435);
        MotorEx follower = new MotorEx(hardwareMap, "follower", RPM_435);

        SimpleServoPivot arm = new SimpleServoPivot(
                ANGLE_COLLECTING,
                ANGLE_DEPOSITING,
                getGoBildaServo(hardwareMap, "arm1"),
                getReversedServo(getGoBildaServo(hardwareMap, "arm2"))
        );

        MotorEx[] motors = {leader, follower};
        waitForStart();

        while (opModeIsActive()) {
            // Read stuff
            bulkReader.bulkRead();

            gamepadEx1.readButtons();

            if (keyPressed(1, Y)) arm.toggle();
            for (MotorEx motor : motors) motor.set(gamepadEx1.getLeftY());

            arm.run();
        }
    }
}
