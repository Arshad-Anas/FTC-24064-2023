package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Class that runs the intake mechanism for in taking pixels
 */
@Config
public final class Intake {
    private final DcMotorEx motor;

    private double motorPower = 0;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void setMotorPower(double motorPower) {
        this.motorPower = motorPower;
    }

    public void run() {
        motor.setPower(motorPower);
    }

//    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
//        telemetry.addData("Roller velocity (ticks/s)", motor.encoder.getCorrectedVelocity());
//    }
}
