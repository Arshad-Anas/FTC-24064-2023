package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static org.firstinspires.ftc.teamcode.opmodes.MainAuton.mTelemetry;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getGoBildaServo;
import static org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot.getReversedServo;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.utilities.SimpleServoPivot;

@Config
public final class Arm {
    private final SimpleServoPivot armPivot, flap;

    boolean hasRetracted = true;

    final ElapsedTime timer = new ElapsedTime();

    // TODO Measure TIME_RETRACT_ARM
    public static double
            ANGLE_COLLECTING = 165,
            ANGLE_DEPOSITING = 80,
            ANGLE_OPEN_FLAP = 90,
            ANGLE_CLOSED_FLAP = 0,
            TIME_RETRACT_ARM = 1;

    public Arm(HardwareMap hardwareMap) {
        flap = new SimpleServoPivot(ANGLE_OPEN_FLAP, ANGLE_CLOSED_FLAP, getGoBildaServo(hardwareMap, "flap"));

        armPivot = new SimpleServoPivot(
                ANGLE_COLLECTING,
                ANGLE_DEPOSITING,
                getGoBildaServo(hardwareMap, "arm1"),
                getReversedServo(getGoBildaServo(hardwareMap, "arm2"))
        );
    }

    public void toggleArm() {
        armPivot.toggle();

        hasRetracted = armPivot.isActivated();
        if (!hasRetracted) {
            timer.reset();
        }
    }

    public void setArm(boolean isDepositing) {
        armPivot.setActivated(isDepositing);
    }

    public void toggleFlap() {
        flap.toggle();
    }

    public void setFlap(boolean isClosed) {
        flap.setActivated(isClosed);
    }

    public void run() {
        flap.updateAngles(ANGLE_OPEN_FLAP, ANGLE_CLOSED_FLAP);
        armPivot.updateAngles(ANGLE_COLLECTING, ANGLE_DEPOSITING);
        flap.run();
        armPivot.run();
    }

    public void printTelemetry() {
        mTelemetry.addData("flap is", (flap.isActivated() ? "closed" : "open"));
        mTelemetry.addData("Arm is", "running to " + (armPivot.isActivated() ? "deposit" : "collect"));
    }
}