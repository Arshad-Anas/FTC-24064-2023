package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_1150;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Class that runs the intake mechanism for in taking pixels
 */
public class Intake {
    private final MotorEx motor;

    private double motorPower = 0;

    public Intake(HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, "intake", RPM_1150);
    }

    public void setMotorPower(double motorPower) {
        this.motorPower = motorPower;
    }

    public void run() {
        motor.set(motorPower);
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Roller velocity (ticks/s)", motor.encoder.getCorrectedVelocity());
    }
}
