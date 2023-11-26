package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private final HardwareMap hw;

    private final Gamepad gamepad;

    public static Servo cServo;

    public static DcMotor masterMotor;

    public static DcMotor slaveMotor;

    public static GamepadEx Gamepad2;

    public Arm (HardwareMap hw, Gamepad gamepad) {
        this.hw = hw;
        this.gamepad = gamepad;
        cServo = hw.get(Servo.class, "cServo");
        masterMotor = hw.get(DcMotor.class, "maMotor");
        slaveMotor = hw.get(DcMotor.class, "slaMotor");

        Gamepad2 = new GamepadEx(gamepad);
    }

    public static void armDeposit() {

        Gamepad2.readButtons();

        if (Gamepad2.getButton(GamepadKeys.Button.RIGHT_BUMPER)) {
            cServo.setPosition(0.6);
        }
    }
}

