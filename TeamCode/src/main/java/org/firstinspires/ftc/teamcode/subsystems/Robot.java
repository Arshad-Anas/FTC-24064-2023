package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.drivetrains.MecanumDrivetrain;

public class Robot {
    public final MecanumDrivetrain drivetrain;
    public final Arm arm;
    public final Intake intake;
    public final Lift lift;

    public Robot(HardwareMap hardwareMap) {
        drivetrain = new MecanumDrivetrain(hardwareMap);
        arm = new Arm(hardwareMap);
        intake = new Intake(hardwareMap);
        lift = new Lift(hardwareMap);
    }

    public void start() {
        drivetrain.start();
    }

    public void interrupt() {
        drivetrain.interrupt();
    }

    public void run() {
        if (lift.getTargetRow() >= 0) arm.extend();
        else arm.retract();

        lift.run();
        arm.run();
        intake.run();
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        arm.printTelemetry(telemetry);
        telemetry.addLine();
        lift.printTelemetry(telemetry);
        telemetry.addLine();
        telemetry.addLine();
        drivetrain.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        intake.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        arm.printNumericalTelemetry(telemetry);
        telemetry.addLine();
        lift.printNumericalTelemetry(telemetry);
    }
}
