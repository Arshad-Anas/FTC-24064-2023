package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_117;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Arm.TIME_RETRACT_ARM;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Lift.TIME_CLOSE_FLAP;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Lift.TIME_EXTEND_ARM;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;

/**
 * Gets all the classes for the robot and calls them with their right parameters
 */
@Config
public final class Robot {

    public static double maxVoltage = 13;
    public final MecanumDrivetrain drivetrain;
    public final Arm arm;
    public final MotorEx intake;
    public final Lift lift;
    public final MotorEx climber;
    private final BulkReader bulkReader;

    /**
     * Constructor of Robot; Instantiates the classes with the hw (hardwareMap)
     * @param hardwareMap; A constant map that holds all the parts for config in code
     */
    public Robot(HardwareMap hardwareMap) {
        bulkReader = new BulkReader(hardwareMap);
        drivetrain = new MecanumDrivetrain(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
        lift = new Lift(hardwareMap);
        climber = new MotorEx(hardwareMap, "climber", RPM_117);
    }

    public void readSensors() {
        bulkReader.bulkRead();
    }

    public void run() {
        if (!arm.hasRetracted && arm.timer.seconds() >= TIME_RETRACT_ARM) {
            arm.setFlap(false);
            lift.retract();
            lift.updateTarget();
            arm.hasRetracted = true;
        }

        if (lift.hasElevated) {
            if (lift.timer.seconds() >= TIME_CLOSE_FLAP) arm.setFlap(true);
            if (lift.timer.seconds() >= TIME_EXTEND_ARM) {
                arm.setArm(true);
                lift.hasElevated = false;
            }
        }

        lift.run();
        arm.run();
    }

    /**
     * Print telemetry data for user debugging
     */
    public void printTelemetry() {
        arm.printTelemetry();
        mTelemetry.addLine();
        lift.printTelemetry();
        mTelemetry.addLine();
        mTelemetry.addLine();
        mTelemetry.addLine();
        drivetrain.printNumericalTelemetry();
        mTelemetry.addLine();
        lift.printNumericalTelemetry();
    }
}
