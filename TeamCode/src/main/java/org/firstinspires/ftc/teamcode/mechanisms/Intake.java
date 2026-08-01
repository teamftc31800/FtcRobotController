package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Intake mechanism module.
 * Handles intake motor control for collecting game elements.
 */
public class Intake {

    private DcMotor intakeMotor = null;
    private boolean hasIntake = false;
    private Telemetry telemetry;

    public static final double POWER_IN = 1.0;
    public static final double POWER_OUT = -1.0;
    public static final double POWER_STOP = 0.0;

    /**
     * Initialize intake with default motor name.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        init(hardwareMap, telemetry, "intake");
    }

    /**
     * Initialize intake with custom motor name.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, String motorName) {
        this.telemetry = telemetry;

        try {
            intakeMotor = hardwareMap.get(DcMotor.class, motorName);
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hasIntake = true;
        } catch (Exception e) {
            hasIntake = false;
        }
    }

    /**
     * Run intake forward (collect).
     */
    public void intake() {
        setPower(POWER_IN);
    }

    /**
     * Run intake reverse (eject).
     */
    public void eject() {
        setPower(POWER_OUT);
    }

    /**
     * Stop intake.
     */
    public void stop() {
        setPower(POWER_STOP);
    }

    /**
     * Set intake power directly.
     */
    public void setPower(double power) {
        if (hasIntake) {
            intakeMotor.setPower(power);
        }
    }

    /**
     * Control intake with gamepad buttons.
     * @param intakeButton Button for intake (e.g., gamepad.a)
     * @param ejectButton Button for eject (e.g., gamepad.b)
     * @param stopButton Button for stop (e.g., gamepad.x)
     */
    public void controlWithButtons(boolean intakeButton, boolean ejectButton, boolean stopButton) {
        if (intakeButton) {
            intake();
        } else if (ejectButton) {
            eject();
        } else if (stopButton) {
            stop();
        }
    }

    /**
     * Report hardware status to telemetry.
     */
    public void reportStatus() {
        telemetry.addData("intake", hasIntake ? "OK" : "MISSING");
    }

    /**
     * Check if intake is available.
     */
    public boolean isAvailable() {
        return hasIntake;
    }
}
