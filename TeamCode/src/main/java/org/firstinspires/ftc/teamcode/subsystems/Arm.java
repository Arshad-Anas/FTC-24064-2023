package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_117;

import static java.lang.Math.cos;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;

public class Arm {

    /**
     * A PIDGains object being set to certain values (tweak these numbers!!)
     */
    public static PIDGains pidGains = new PIDGains(
            0,
            0,
            0,
            Double.POSITIVE_INFINITY
    );

    // Sets the filter for PID outputs and constrains overshoots with controlling (also tweak!!)
    public static LowPassGains filterGains = new LowPassGains(0, 2);

    // TODO Measure

    /**
     * Sets the constants for the positions, conversions, etc
     * Remember to set these constants correctly!
     */
    public static double
            DEPOSIT_POSITION = 0,
            COLLECT_POSITION = 0,
            TICKS_TO_DEGREES = 0,
            kG = 0;

    private final MotorEx motor;

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    private final VoltageSensor batteryVoltageSensor;

    private State currentState = new State();

    private boolean isExtended = false;

    /**
     * Constructor for arm class; Sets variables with hw (hardwareMap)
     * @param hardwareMap; A constant map that holds all the parts for config in code
     */
    public Arm (HardwareMap hardwareMap) {
        motor = new MotorEx(hardwareMap, "arm", RPM_117);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void extend() {
        isExtended = true;
    }

    public void retract() {
        isExtended = false;
    }

    /**
     * Method that is mostly used; Has all the calculations
     * Sets the three variables; The currentState sets to the position of the motor; Other two set the constants of the gains
     * Sets the target of the controller to a state, which is set to either the DEPOSIT_POSITION or COLLECT_POSITION
     * Sets the motor's speed to the calculation of the current state (outputs a proportional number)
     * Converts the ticks into degrees, and takes into account of the batteryVoltageSensor
     */
    public void run() {
        currentState = new State(motor.encoder.getPosition());
        controller.setGains(pidGains);
        derivFilter.setGains(filterGains);

        controller.setTarget(new State(isExtended ? DEPOSIT_POSITION : COLLECT_POSITION));
        motor.set(
                controller.calculate(currentState) +
                        cos(toRadians(currentState.x * TICKS_TO_DEGREES)) * kG * (12 / batteryVoltageSensor.getVoltage())
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

