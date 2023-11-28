package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1150;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    MotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, "intake", RPM_1150);
    }

    public void run(double power) {
        motor.set(power);
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Roller velocity (ticks/s)", motor.encoder.getCorrectedVelocity());
    }
}
