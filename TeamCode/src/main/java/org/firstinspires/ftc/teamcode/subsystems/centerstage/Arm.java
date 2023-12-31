package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@Config
public class Arm {
    private final SimpleServoPivot armPivot;
    private final SimpleServoPivot flap;

    private boolean isDepositing = false;
    private boolean isFlapClosed = false;

    // TODO MEASURE
    public static double
            ANGLE_DOWN_ARM = 155,
            ANGLE_UP_ARM = 70,
            ANGLE_OPEN_FLAP = 90,
            ANGLE_CLOSED_FLAP = 0;

    public Arm(HardwareMap hardwareMap) {
        flap = new SimpleServoPivot(ANGLE_OPEN_FLAP, ANGLE_CLOSED_FLAP, getGoBildaServo(hardwareMap, "flap"));

        armPivot = new SimpleServoPivot(
                ANGLE_DOWN_ARM,
                ANGLE_UP_ARM,
                getGoBildaServo(hardwareMap, "arm1"),
                getReversedServo(getGoBildaServo(hardwareMap, "arm2"))
        );
    }

    public void toggleArm() {
        setExtended(!isDepositing);
    }

    public void setExtended(boolean isDepositing) {
        this.isDepositing = isDepositing;
        armPivot.setActivated(isDepositing);
    }

    public void toggleFlap() {
        setFlapClosed(!isFlapClosed);
    }

    public void setFlapClosed(boolean isClosed) {
        this.isFlapClosed = isClosed;
        flap.setActivated(isClosed);
    }

    public void run() {
        flap.updateAngles(ANGLE_OPEN_FLAP, ANGLE_CLOSED_FLAP);
        armPivot.updateAngles(ANGLE_DOWN_ARM, ANGLE_UP_ARM);
        flap.run();
        armPivot.run();
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("flap is", (flap.getActivated() ? "closed" : "open"));
        telemetry.addData("Arm is", "running to " + (armPivot.getActivated() ? "deposit" : "collect"));
    }
}