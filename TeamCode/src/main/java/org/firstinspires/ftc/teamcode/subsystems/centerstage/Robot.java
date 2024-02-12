package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_117;
import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Arm.TIME_DEPOSIT_1_PIXEL;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.utilities.BulkReader;
import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

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
    public final SimpleServoPivot launcher;
    public final SimpleServoPivot launcherClamp;
    public final SimpleServoPivot deployableRoller;
    public final SimpleServoPivot wrist;
    private final BulkReader bulkReader;

    private static double
            ANGLE_DRONE_LOAD = 180,
            ANGLE_DRONE_LAUNCH = 0,
            ANGLE_DRONE_CLAMP = 90,
            ANGLE_DRONE_UNCLAMPED = 0,
            ANGLE_UNDEPLOYED = 90,
            ANGLE_DEPLOYED = 0,
            ANGLE_WRIST_UNDEPLOYED = 0,
            ANGLE_WRIST_DEPLOYED = 15;

    /**
     * Constructor of Robot; Instantiates the classes with the hardwareMap
     * @param hardwareMap; A constant map that holds all the parts for config in code
     */
    public Robot(HardwareMap hardwareMap) {
        bulkReader = new BulkReader(hardwareMap);
        drivetrain = new MecanumDrivetrain(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new MotorEx(hardwareMap, "intake", Motor.GoBILDA.RPM_1620);
        lift = new Lift(hardwareMap);
        launcher = new SimpleServoPivot(ANGLE_DRONE_LOAD, ANGLE_DRONE_LAUNCH, getGoBildaServo(hardwareMap, "launcher"));
        launcherClamp = new SimpleServoPivot(ANGLE_DRONE_CLAMP, ANGLE_DRONE_UNCLAMPED, getGoBildaServo(hardwareMap, "launcher-clamp"));

        deployableRoller = new SimpleServoPivot(
                ANGLE_UNDEPLOYED,
                ANGLE_DEPLOYED,
                getGoBildaServo(hardwareMap, "roller1"),
                getReversedServo(getGoBildaServo(hardwareMap, "roller2"))
        );

        wrist = new SimpleServoPivot(
                ANGLE_WRIST_UNDEPLOYED,
                ANGLE_WRIST_DEPLOYED,
                getGoBildaServo(hardwareMap, "wrist1"),
                getReversedServo(getGoBildaServo(hardwareMap, "wrist2"))
        );
    }

    public void readSensors() {
        bulkReader.bulkRead();
    }

    public void run() {
        if (lift.getSetPoint() == -1) {
            arm.setFlap(intake.get() == 0);
        } else {
            if (arm.flapTimerCondition && arm.flapTimer.seconds() >= TIME_DEPOSIT_1_PIXEL) {
                arm.setFlap(true);
                arm.flapTimerCondition = false;
            }
        }

        wrist.run();
        deployableRoller.run();
        launcherClamp.run();
        launcher.run();
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
