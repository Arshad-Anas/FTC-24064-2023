package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.opmodes.centerstage.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Arm.TIME_DEPOSIT_1_PIXEL;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.config.Config;
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
    public final Lift lift;
    public final SimpleServoPivot launcher;
    public final SimpleServoPivot launcherClamp;
    public final Rollers rollers;
    public final SimpleServoPivot wrist;
    public final SimpleServoPivot purplePixel;
    private final BulkReader bulkReader;

    public static double
            ANGLE_DRONE_LOAD = 140,
            ANGLE_DRONE_LAUNCH = 0,
            ANGLE_DRONE_CLAMP = 90,
            ANGLE_DRONE_UNCLAMPED = 0,
            ANGLE_WRIST_UNDEPLOYED = 60,
            ANGLE_WRIST_DEPLOYED = 45,
            ANGLE_PURPLE_PIXEL_UNDEPLOYED = 0,
            ANGLE_PURPLE_PIXEL_DEPLOYED = 90;

    /**
     * Constructor of Robot; Instantiates the classes with the hardwareMap
     * @param hardwareMap; A constant map that holds all the parts for config in code
     */
    public Robot(HardwareMap hardwareMap) {
        bulkReader = new BulkReader(hardwareMap);
        drivetrain = new MecanumDrivetrain(hardwareMap);
        arm = new Arm(hardwareMap);
        lift = new Lift(hardwareMap);
        launcher = new SimpleServoPivot(ANGLE_DRONE_LOAD, ANGLE_DRONE_LAUNCH, getGoBildaServo(hardwareMap, "launcher"));
        launcherClamp = new SimpleServoPivot(ANGLE_DRONE_CLAMP, ANGLE_DRONE_UNCLAMPED, getGoBildaServo(hardwareMap, "launcher-clamp"));

        rollers = new Rollers(hardwareMap);

        wrist = new SimpleServoPivot(
                ANGLE_WRIST_UNDEPLOYED,
                ANGLE_WRIST_DEPLOYED,
                getGoBildaServo(hardwareMap, "wrist1"),
                getReversedServo(getGoBildaServo(hardwareMap, "wrist2"))
        );

        purplePixel = new SimpleServoPivot(
                ANGLE_PURPLE_PIXEL_UNDEPLOYED,
                ANGLE_PURPLE_PIXEL_DEPLOYED,
                getGoBildaServo(hardwareMap, "purple placer")
        );
    }

    public void readSensors() {
        bulkReader.bulkRead();
    }

    public void run() {
        if (lift.getSetPoint() == -1) {
            arm.setFlap(rollers.intakePower() == 0);
        } else {
            if (arm.flapTimerCondition && arm.flapTimer.seconds() >= TIME_DEPOSIT_1_PIXEL) {
                arm.setFlap(true);
                arm.flapTimerCondition = false;
            }
        }

        rollers.run();
        purplePixel.run();
        wrist.run();
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
        rollers.printTelemetry();
        mTelemetry.addLine();
        mTelemetry.addLine();
        mTelemetry.addLine();
        drivetrain.printNumericalTelemetry();
        mTelemetry.addLine();
        lift.printNumericalTelemetry();
    }
}
