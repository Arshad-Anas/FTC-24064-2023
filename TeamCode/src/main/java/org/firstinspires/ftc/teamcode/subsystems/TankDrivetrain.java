package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.drive.Drive;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.Arrays;
import java.util.Collections;


public class TankDrivetrain {

    private final DifferentialDrive tankDrivetrain;

    protected final MotorEx[] motors;

    private final HardwareMap hw;
    private final double motorCPR, motorRPM;

    protected final VoltageSensor batteryVoltageSensor;

    protected MotorEx getMotor(String name) {
        return new MotorEx(hw, name, motorCPR, motorRPM);
    }

    public TankDrivetrain(HardwareMap hw, double motorCPR, double motorRPM) {
        this.hw = hw;
        this.motorCPR = motorCPR;
        this.motorRPM = motorRPM;
        this.batteryVoltageSensor = hw.voltageSensor.iterator().next();

        // Assign motors using their hardware map names, each drive-type can have different names if needed
        motors = new MotorEx[]{
                getMotor("left"),
                getMotor("right"),
        };

        for (MotorEx motor : motors) motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        // Initialize the FTCLib drive-base
        tankDrivetrain = new DifferentialDrive(false, motors[0], motors[1]);

        resetPosition();
    }

    public static double normalizeAngle(double angle) {
        angle %= 360.0;
        if (angle <= -180.0) return angle + 360.0;
        if (angle > 180.0) return angle - 360.0;
        if (angle == -0.0) return 0.0;
        return angle;
    }

    /**
     * Set internal heading of the robot to correct field-centric direction
     *
     * @param angle Angle of the robot in degrees, 0 facing forward and increases counter-clockwise
     */

    public int getMotorPos(int motorIndex) {
        return motors[motorIndex].encoder.getPosition();
    }

    public double getY() {
        return (getMotorPos(0) + getMotorPos(1) + getMotorPos(2) + getMotorPos(3)) * 0.25;
    }

    public double getX() {
        return (getMotorPos(0) - getMotorPos(1) - getMotorPos(2) + getMotorPos(3)) * 0.25;
    }

    public void resetPosition() {
        for (MotorEx motor : motors) motor.encoder.reset();
    }

    public void run(double xCommand, double yCommand, double turnCommand) {
        // normalize inputs
        double max = Collections.max(Arrays.asList(xCommand, yCommand, turnCommand, 1.0));
        xCommand /= max;
        yCommand /= max;
        turnCommand /= max;

        tankDrivetrain.driveFieldCentric(xCommand, yCommand, turnCommand, getHeading());
    }
}

