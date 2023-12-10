package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

public class Carriage {
    private final SimpleServoPivot servoPivot;

    private boolean isDepositing = false;

    // TODO MEASURE
    private static final double
            COLLECTING_ANGLE = 0,
            DEPOSITING_ANGLE = 0,
            ARM_POS_TO_START_DEPOSIT = 0;

    public Carriage(HardwareMap hardwareMap) {
        servoPivot = new SimpleServoPivot(COLLECTING_ANGLE, DEPOSITING_ANGLE, SimpleServoPivot.getGoBildaServo(hardwareMap, "carriage"));
    }

    public void set(double armRotation) {
        set(armRotation >= ARM_POS_TO_START_DEPOSIT);
    }

    public void set(boolean isDepositing) {
        this.isDepositing = isDepositing;
        servoPivot.setActivated(isDepositing);
    }

    public void run() {
        servoPivot.run();
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Carriage is", "running to " + (isDepositing ? "deposit" : "collect"));
    }
}
