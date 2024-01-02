package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Arm.TIME_RETRACT_ARM;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Lift.TIME_CLOSE_FLAP;
import static org.firstinspires.ftc.teamcode.subsystems.centerstage.Lift.TIME_EXTEND_ARM;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.MecanumDrivetrain;

import java.util.List;

/**
 * Gets all the classes for the robot and calls them with their right parameters
 */
@Config
public final class Robot {

    public static double maxVoltage = 13;
    public final MecanumDrivetrain drivetrain;
    public final Arm arm;
    public final Intake intake;
    public final Lift lift;
    private final List<LynxModule> revHubs;

    /**
     * Constructor of Robot; Instantiates the classes with the hw (hardwareMap)
     * @param hardwareMap; A constant map that holds all the parts for config in code
     */
    public Robot(HardwareMap hardwareMap) {
        drivetrain = new MecanumDrivetrain(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);

        revHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : revHubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
    }

    public void interrupt() {
        drivetrain.interrupt();
    }

    public void readSensors() {
        for (LynxModule hub : revHubs) hub.clearBulkCache();
    }

    public void run() {
        if (!arm.hasRetracted && arm.timer.seconds() >= TIME_RETRACT_ARM) {
            arm.setFlap(false);
            lift.setTargetRow(-1);
            lift.updateTarget();
            arm.hasRetracted = true;
        }

        if (lift.hasElevated) {
            if (lift.timer.seconds() >= TIME_EXTEND_ARM) {
                arm.setArm(true);
                lift.hasElevated = false;
            }
            if (lift.timer.seconds() >= TIME_CLOSE_FLAP) arm.setFlap(true);
        }

        lift.run();
        arm.run();
        intake.run();
    }

    /**
     * Print telemetry data for user debugging
     * @param telemetry; Where the data is stored
     */
    public void printTelemetry(MultipleTelemetry telemetry) {
        arm.printTelemetry(telemetry);
        telemetry.addLine();
        lift.printTelemetry(telemetry);
        telemetry.addLine();
        telemetry.addLine();
        telemetry.addLine();
        drivetrain.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        lift.printNumericalTelemetry(telemetry);
    }
}
