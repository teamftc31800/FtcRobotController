package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Flywheel shooter mechanism with feeder motor and servos.
 * Handles velocity control, RPM targeting, and feeder synchronization.
 */
public class Flywheel {

    private DcMotorEx flywheelMotor = null;
    private DcMotorEx feederMotor = null;
    private CRServo feederServoLeft = null;
    private CRServo feederServoRight = null;

    private boolean hasFlywheel = false;
    private boolean hasFeederMotor = false;
    private boolean hasFeederServoLeft = false;
    private boolean hasFeederServoRight = false;

    private Telemetry telemetry;

    // Constants
    private static final double FLYWHEEL_TPR = 28.0;
    private static final double FEEDER_TPR = 537.7;
    private static final double DEFAULT_FEEDER_RPM = 400.0;

    // Configurable settings
    private double targetRPM = 2750.0;
    private double toleranceRPM = 200.0;
    private int toleranceCountRequired = 1;

    // State tracking
    private boolean active = false;
    private int inToleranceCount = 0;
    private double largestOvershoot = 0;

    /**
     * Initialize flywheel with default hardware names.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        init(hardwareMap, telemetry, "launcher", "feeder", "feederServoLeft", "feederServoRight");
    }

    /**
     * Initialize flywheel with custom hardware names.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry,
                     String flywheelName, String feederMotorName,
                     String leftServoName, String rightServoName) {
        this.telemetry = telemetry;

        // Flywheel motor
        try {
            flywheelMotor = hardwareMap.get(DcMotorEx.class, flywheelName);
            flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheelMotor.setDirection(DcMotor.Direction.REVERSE);
            hasFlywheel = true;
        } catch (Exception e) {
            hasFlywheel = false;
        }

        // Feeder motor
        try {
            feederMotor = hardwareMap.get(DcMotorEx.class, feederMotorName);
            feederMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            feederMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            feederMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            feederMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            hasFeederMotor = true;
        } catch (Exception e) {
            hasFeederMotor = false;
        }

        // Left feeder servo
        try {
            feederServoLeft = hardwareMap.get(CRServo.class, leftServoName);
            feederServoLeft.setPower(0.0);
            hasFeederServoLeft = true;
        } catch (Exception e) {
            hasFeederServoLeft = false;
        }

        // Right feeder servo
        try {
            feederServoRight = hardwareMap.get(CRServo.class, rightServoName);
            feederServoRight.setPower(0.0);
            hasFeederServoRight = true;
        } catch (Exception e) {
            hasFeederServoRight = false;
        }
    }

    /**
     * Set flywheel motor direction.
     */
    public void setDirection(DcMotor.Direction direction) {
        if (hasFlywheel) {
            flywheelMotor.setDirection(direction);
        }
    }

    /**
     * Set target RPM for the flywheel.
     */
    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    /**
     * Get current target RPM.
     */
    public double getTargetRPM() {
        return targetRPM;
    }

    /**
     * Set RPM tolerance for determining "at speed".
     */
    public void setTolerance(double toleranceRPM, int requiredCount) {
        this.toleranceRPM = toleranceRPM;
        this.toleranceCountRequired = requiredCount;
    }

    /**
     * Activate the flywheel system.
     */
    public void activate() {
        active = true;
        inToleranceCount = 0;
    }

    /**
     * Deactivate the flywheel system.
     */
    public void deactivate() {
        active = false;
        inToleranceCount = 0;
        if (hasFlywheel) {
            flywheelMotor.setPower(0.0);
        }
        if (hasFeederMotor) {
            feederMotor.setPower(0.0);
        }
    }

    /**
     * Toggle flywheel active state.
     */
    public void toggle() {
        if (active) {
            deactivate();
        } else {
            activate();
        }
    }

    /**
     * Check if flywheel is active.
     */
    public boolean isActive() {
        return active;
    }

    /**
     * Update flywheel control loop. Call this in your OpMode loop().
     * @return true if flywheel is at target speed
     */
    public boolean update() {
        if (!hasFlywheel || !active) {
            return false;
        }

        // Spin up flywheel
        double targetTPS = rpmToTicksPerSec(targetRPM, FLYWHEEL_TPR);
        flywheelMotor.setVelocity(targetTPS);

        // Read actual RPM
        double currentTPS = flywheelMotor.getVelocity();
        double currentRPM = ticksPerSecToRPM(currentTPS, FLYWHEEL_TPR);
        boolean atSpeed = Math.abs(currentRPM - targetRPM) <= toleranceRPM;

        // Track overshoot
        double overshoot = currentRPM - targetRPM;
        if (overshoot > largestOvershoot) {
            largestOvershoot = overshoot;
        }

        // Debounce
        if (atSpeed) {
            inToleranceCount = Math.min(inToleranceCount + 1, toleranceCountRequired);
        } else {
            inToleranceCount = 0;
        }

        // Run feeder when at speed
        if (hasFeederMotor && inToleranceCount >= toleranceCountRequired) {
            feederMotor.setVelocity(rpmToTicksPerSec(DEFAULT_FEEDER_RPM, FEEDER_TPR));
        }

        return inToleranceCount >= toleranceCountRequired;
    }

    /**
     * Control left feeder servo.
     */
    public void setLeftServoPower(double power) {
        if (hasFeederServoLeft) {
            feederServoLeft.setPower(power);
        }
    }

    /**
     * Control right feeder servo.
     */
    public void setRightServoPower(double power) {
        if (hasFeederServoRight) {
            feederServoRight.setPower(power);
        }
    }

    /**
     * Control both feeder servos with D-pad.
     */
    public void controlServosWithDpad(boolean dpadLeft, boolean dpadRight) {
        setLeftServoPower(dpadLeft ? -1.0 : 0.0);
        setRightServoPower(dpadRight ? -1.0 : 0.0);
    }

    /**
     * Get current flywheel RPM.
     */
    public double getCurrentRPM() {
        if (!hasFlywheel) return 0;
        double currentTPS = flywheelMotor.getVelocity();
        return ticksPerSecToRPM(currentTPS, FLYWHEEL_TPR);
    }

    /**
     * Check if flywheel is at target speed.
     */
    public boolean isAtSpeed() {
        return inToleranceCount >= toleranceCountRequired;
    }

    /**
     * Get largest overshoot recorded.
     */
    public double getLargestOvershoot() {
        return largestOvershoot;
    }

    /**
     * Report hardware status and telemetry.
     */
    public void reportStatus() {
        telemetry.addData("flywheel", hasFlywheel ? "OK" : "MISSING");
        telemetry.addData("feederMotor", hasFeederMotor ? "OK" : "MISSING");
        telemetry.addData("feederServoLeft", hasFeederServoLeft ? "OK" : "MISSING");
        telemetry.addData("feederServoRight", hasFeederServoRight ? "OK" : "MISSING");
    }

    /**
     * Report flywheel telemetry during operation.
     */
    public void reportTelemetry() {
        if (hasFlywheel) {
            double currentRPM = getCurrentRPM();
            boolean atSpeed = isAtSpeed();
            telemetry.addData("Flywheel", "Target %.0f RPM | Now %.0f RPM %s",
                    targetRPM, currentRPM, atSpeed ? "(at speed)" : "");
            telemetry.addData("Largest Overshoot", largestOvershoot);
        }
    }

    // Math helpers
    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        return (rpm * ticksPerRev) / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec, double ticksPerRev) {
        return (ticksPerSec * 60.0) / ticksPerRev;
    }
}
