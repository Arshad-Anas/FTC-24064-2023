package org.firstinspires.ftc.teamcode.subsystems;

import static com.arcrobotics.ftclib.hardware.motors.Motor.GoBILDA.RPM_435;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.control.State;
import org.firstinspires.ftc.teamcode.control.controllers.PIDController;
import org.firstinspires.ftc.teamcode.control.filters.FIRLowPassFilter;
import org.firstinspires.ftc.teamcode.control.gainmatrices.LowPassGains;
import org.firstinspires.ftc.teamcode.control.gainmatrices.PIDGains;

public class Lift {

    public static PIDGains pidGains = new PIDGains(
            0,
            0,
            0,
            Double.POSITIVE_INFINITY
    );

    public static LowPassGains filterGains = new LowPassGains(0, 2);

    // TODO Measure (ticks)
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

    public Lift(HardwareMap hardwareMap) {
        MotorEx leader = new MotorEx(hardwareMap, "leader", RPM_435);
        MotorEx follower = new MotorEx(hardwareMap, "follower", RPM_435);

        motors = new MotorEx[]{leader, follower};
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    // TODO Implement this!
    public void setTargetRow(int targetRow) {
        this.targetRow = Math.min(targetRow, 10);
        State targetState = new State(this.targetRow < 0 ? 0 : this.targetRow * PIXEL_HEIGHT + BOTTOM_ROW_HEIGHT);
        controller.setTarget(targetState);
    }

    public void increment() {
        setTargetRow(targetRow + 1);
    }

    public void decrement() {
        setTargetRow(targetRow - 1);
    }

    public void run() {
        currentState = new State(motors[0].encoder.getPosition());
        controller.setGains(pidGains);
        derivFilter.setGains(filterGains);

        run(controller.calculate(currentState), false);
    }

    public void run(double motorPower) {
        run(motorPower, true);
    }

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
