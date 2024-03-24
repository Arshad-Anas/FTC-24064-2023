package org.firstinspires.ftc.teamcode.subsystems.utilities.sensors;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.subsystems.utilities.AveragingBuffer;

public final class HeadingIMU {
    private final IMU imu;

    private double heading, angularVel;
    private final AveragingBuffer headingBuffer, angularVelBuffer;

    public HeadingIMU(HardwareMap hardwareMap, String name, RevHubOrientationOnRobot imuOrientation) {
        imu = hardwareMap.get(IMU.class, name);
        imu.resetDeviceConfigurationForOpMode();
        imu.resetYaw();
        imu.initialize(new IMU.Parameters(imuOrientation));

        headingBuffer = new AveragingBuffer(10);
        angularVelBuffer = new AveragingBuffer(10);
    }

    public void update() {
        heading = headingBuffer.put(imu.getRobotYawPitchRollAngles().getYaw(RADIANS));
        angularVel = angularVelBuffer.put(imu.getRobotAngularVelocity(RADIANS).zRotationRate);
    }

    public double getHeading() {
        return heading;
    }

    public double getAngularVel() {
        return angularVel;
    }
}
