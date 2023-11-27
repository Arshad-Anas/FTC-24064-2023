package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_117;

import static java.lang.Math.cos;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;

public class Arm {

    public static PIDGains pidGains = new PIDGains(
            0,
            0,
            0,
            Double.POSITIVE_INFINITY
    );

    public static LowPassGains filterGains = new LowPassGains(0, 2);

    // TODO Measure
    public static double
            DEPOSIT_POSITION = 0,
            COLLECT_POSITION = 0,
            TICKS_TO_DEGREES = 0,
            K_COS = 0;

    private final MotorEx motor;

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    private State currentState = new State();

    private boolean isExtended = false;

    public Arm (HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, "arm", RPM_117);
    }

    public void extend() {
        isExtended = true;
    }

    public void retract() {
        isExtended = false;
    }

    public void run() {
        currentState = new State(motor.encoder.getPosition());
        controller.setGains(pidGains);
        derivFilter.setGains(filterGains);

        controller.setTarget(new State(isExtended ? DEPOSIT_POSITION : COLLECT_POSITION));
        motor.set(
                controller.calculate(currentState) +
                        cos(toRadians(currentState.x * TICKS_TO_DEGREES)) * K_COS
        );
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Arm is", (isExtended ? "" : "not") + " extended");
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Current position (ticks)", currentState.x);
        telemetry.addData("Error derivative (ticks/s)", controller.getErrorDerivative());
        telemetry.addData("Current angle (degrees)", currentState.x * TICKS_TO_DEGREES);
    }
}

