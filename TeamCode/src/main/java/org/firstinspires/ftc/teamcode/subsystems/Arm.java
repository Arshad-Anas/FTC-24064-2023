package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {

    private Servo carriageServo;

    private DcMotor masterMotor, slaveMotor;

    private GamepadEx gamepad2;

    public Arm (HardwareMap hardwareMap) {
        carriageServo = hardwareMap.get(Servo.class, "cServo");
        masterMotor = hardwareMap.get(DcMotor.class, "maMotor");
        slaveMotor = hardwareMap.get(DcMotor.class, "slaMotor");
    }

    public void deposit() {
        carriageServo.setPosition(0.6);
    }
}

