package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public final class Rollers {
    private final MotorEx intake;
    private final SimpleServo deployableRoller;


    public static double ANGLE_DEPLOYABLE_ROLLER = 108;
    public double setPoint = ANGLE_DEPLOYABLE_ROLLER;

    public Rollers(HardwareMap hardwareMap) {
        deployableRoller = getGoBildaServo(hardwareMap, "roller1");
        intake = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
    }

    public void intake(double power) {
        intake.set(power);
    }

    public double intakePower() {
        return intake.get();
    }

    public void setDeployableWithTrigger(double trigger) {
        setDeployable(trigger < 0 ? ANGLE_DEPLOYABLE_ROLLER + ANGLE_DEPLOYABLE_ROLLER * trigger : ANGLE_DEPLOYABLE_ROLLER);
    }

    public void setDeployable(double angle) {
        setPoint = angle;
    }

    public void run() {
        deployableRoller.turnToAngle(setPoint);
    }

    public void printTelemetry() {
        mTelemetry.addData("Roller angle (degrees)", setPoint);
    }
}
