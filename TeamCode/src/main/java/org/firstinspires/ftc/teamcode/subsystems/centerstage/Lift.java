package org.firstinspires.ftc.teamcode.subsystems.centerstage;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_435;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.motion.State;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;

public class Lift {

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

    // TODO Measure (ticks)
    /**
     * Sets the constants for the positions, conversions, etc
     * Remember to set these constants correctly! (in ticks)
     */
    public static double
            BOTTOM_ROW_HEIGHT = 100,
            PIXEL_HEIGHT = 10,
            kG = 0;

    private final MotorEx[] motors;

    private final FIRLowPassFilter derivFilter = new FIRLowPassFilter(filterGains);
    private final PIDController controller = new PIDController(derivFilter);

    private final VoltageSensor batteryVoltageSensor;

    private State currentState = new State();

    private int targetRow = -1;

    /**
     * Constructor of Lift class; Sets variables with hw (hardwareMap)
     * @param hardwareMap; A constant map that holds all the parts for config in code
     */
    public Lift(HardwareMap hardwareMap) {
        MotorEx leader = new MotorEx(hardwareMap, "leader", RPM_435);
        MotorEx follower = new MotorEx(hardwareMap, "follower", RPM_435);

        motors = new MotorEx[]{leader, follower};
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    /**
     * Uses the targetRow variable to find out the measurement for where the pixel should be placed
     * @param targetRow; The set target row on the backstage that the arm is suspected to drop the pixel
     */
    // TODO Implement this!
    public void setTargetRow(int targetRow) {
        this.targetRow = max(min(targetRow, 10), -1);
        State targetState = new State(this.targetRow < 0 ? 0 : this.targetRow * PIXEL_HEIGHT + BOTTOM_ROW_HEIGHT);
        controller.setTarget(targetState);
    }

    public int getTargetRow() {
        return this.targetRow;
    }

    public void increment() {
        setTargetRow(targetRow + 1);
    }

    public void decrement() {
        setTargetRow(targetRow - 1);
    }

    /**
     * Sets the three variables; The currentState sets to the position of the motor; Other two set the constants of the gains
     * Calls another run() method that calculates the motor output proportionally and doesn't compensate for power
     */
    public void run() {
        currentState = new State(motors[0].encoder.getPosition());
        controller.setGains(pidGains);
        derivFilter.setGains(filterGains);

        run(controller.calculate(currentState), false);
    }

    public void run(double motorPower) {
        run(motorPower, true);
    }

    /**
     * Checks if the voltage is to be saved, if true, use scalar to modify motor power
     * @param motorPower; The power that is set to be used to set the motor's power
     * @param voltageCompensate; Boolean that is used if battery is low, and if it needs to compensate (save)
     */
    private void run(double motorPower, boolean voltageCompensate) {
        double scalar = 12 / batteryVoltageSensor.getVoltage();

        if (voltageCompensate) motorPower *= scalar;

        for (MotorEx motor : motors) motor.set(motorPower + kG * scalar);
    }

    public void printTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Target position (pixels)", targetRow < 0 ? "Retracted" : "Row " + targetRow);
    }

    public void printNumericalTelemetry(MultipleTelemetry telemetry) {
        telemetry.addData("Current position (ticks)", currentState.x);
        telemetry.addData("Error derivative (ticks/s)", controller.getErrorDerivative());
    }
}
