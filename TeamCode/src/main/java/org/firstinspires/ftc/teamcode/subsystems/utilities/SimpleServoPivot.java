package org.firstinspires.ftc.teamcode.subsystems.utilities;


import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Servo(s) with two set positions <p>
 * Controlled by {@link #toggle} and {@link #setActivated}
 *
 * @author Arshad Anas
 * @since 2023/06/14
 */
public class SimpleServoPivot {

    public static SimpleServo getReversedServo(SimpleServo servo) {
        servo.setInverted(true);
        return servo;
    }

    public static SimpleServo getAxonServo(HardwareMap hardwareMap, String name) {
        return new SimpleServo(hardwareMap, name, 0, 355);
    }

    public static SimpleServo getGoBildaServo(HardwareMap hardwareMap, String name) {
        return new SimpleServo(hardwareMap, name, 0, 300);
    }

    private final SimpleServo[] servos;

    private double INITIAL_ANGLE, ACTIVATED_ANGLE;

    private boolean activated = false;

    public SimpleServoPivot(double INITIAL_ANGLE, double ACTIVATED_ANGLE, SimpleServo... servos) {
        this.servos = servos;
        updateAngles(INITIAL_ANGLE, ACTIVATED_ANGLE);
    }

    public void updateAngles(double INITIAL_ANGLE, double ACTIVATED_ANGLE) {
        this.INITIAL_ANGLE = INITIAL_ANGLE;
        this.ACTIVATED_ANGLE = ACTIVATED_ANGLE;
    }

    /**
     * Toggles the state of the {@link #servos}
     */
    public void toggle() {
        setActivated(!activated);
    }

    /**
     * Set state of the {@link #servos}
     *
     * @param activated False for position A, true for position B
     */
    public void setActivated(boolean activated) {
        this.activated = activated;
    }

    /**
     * Get state of the {@link #servos} <p>
     * False if position A (default) <p>
     * True if in position B
     */
    public boolean isActivated() {
        return activated;
    }

    /**
     * Hold {@link #servos} position
     */
    public void run() {
        for (SimpleServo servo : servos) servo.turnToAngle(activated ? ACTIVATED_ANGLE : INITIAL_ANGLE);
    }
}
