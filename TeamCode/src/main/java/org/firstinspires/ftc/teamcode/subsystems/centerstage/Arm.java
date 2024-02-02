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

    boolean flapTimerCondition = false;

    public final ElapsedTime flapTimer = new ElapsedTime();

    // TODO Measure TIME_RETRACT_ARM
    public static double
            ANGLE_COLLECTING = 80,
            ANGLE_DEPOSITING = 167,
            ANGLE_OPEN_FLAP = 90,
            ANGLE_CLOSED_FLAP = 0,
            TIME_DEPOSIT_1_PIXEL = 0.152;

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
    }

    public void toggleFlap() {
        flap.toggle();

        flapTimerCondition = !flap.isActivated();
        if (flapTimerCondition) {
            flapTimer.reset();
        }
    }

    public void setFlap(boolean isClosed) {
        flap.setActivated(isClosed);
    }

    public boolean isArmActivated() {
        return armPivot.isActivated();
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