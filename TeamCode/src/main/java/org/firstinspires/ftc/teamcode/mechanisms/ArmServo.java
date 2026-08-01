package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Arm servo mechanism with angle-based control and presets.
 * Handles position control, bumper stepping, and angle mapping.
 */
public class ArmServo {

    private Servo armServo = null;
    private boolean hasArmServo = false;
    private Telemetry telemetry;

    // Servo position limits
    private double servoMin = 0.0;
    private double servoMax = 1.0;
    private double servoStep = 0.1;

    // Angle mapping
    private double armMinDeg = 0.0;
    private double armMaxDeg = 300.0;

    // Edge detection for bumper control
    private boolean lastRightBumper = false;
    private boolean lastLeftBumper = false;

    /**
     * Initialize arm servo with default name.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        init(hardwareMap, telemetry, "armServo");
    }

    /**
     * Initialize arm servo with custom name.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry, String servoName) {
        this.telemetry = telemetry;

        try {
            armServo = hardwareMap.get(Servo.class, servoName);
            armServo.scaleRange(0.2, 0.8);
            armServo.setPosition(servoMin);
            hasArmServo = true;
        } catch (Exception e) {
            hasArmServo = false;
        }
    }

    /**
     * Configure servo range and step size.
     */
    public void configure(double min, double max, double step) {
        this.servoMin = min;
        this.servoMax = max;
        this.servoStep = step;
    }

    /**
     * Configure angle mapping.
     */
    public void setAngleRange(double minDeg, double maxDeg) {
        this.armMinDeg = minDeg;
        this.armMaxDeg = maxDeg;
    }

    /**
     * Set servo position directly (0.0 to 1.0).
     */
    public void setPosition(double position) {
        if (hasArmServo) {
            position = Math.max(servoMin, Math.min(servoMax, position));
            armServo.setPosition(position);
        }
    }

    /**
     * Set arm angle in degrees.
     */
    public void setAngle(double degrees) {
        setPosition(degToServoPos(degrees));
    }

    /**
     * Get current servo position.
     */
    public double getPosition() {
        if (hasArmServo) {
            return armServo.getPosition();
        }
        return 0;
    }

    /**
     * Get current angle in degrees.
     */
    public double getAngle() {
        return servoPosToDeg(getPosition());
    }

    /**
     * Control arm with bumpers (one step per press).
     * @param rightBumper Increase position
     * @param leftBumper Decrease position
     */
    public void controlWithBumpers(boolean rightBumper, boolean leftBumper) {
        if (!hasArmServo) return;

        double servoPos = armServo.getPosition();

        // Rising edge detect
        if (rightBumper && !lastRightBumper) {
            servoPos += servoStep;
        } else if (leftBumper && !lastLeftBumper) {
            servoPos -= servoStep;
        }

        lastRightBumper = rightBumper;
        lastLeftBumper = leftBumper;

        // Clamp and apply
        servoPos = Math.max(servoMin, Math.min(servoMax, servoPos));
        armServo.setPosition(servoPos);
    }

    /**
     * Convert servo position (0-1) to degrees.
     */
    public double servoPosToDeg(double pos) {
        pos = Math.max(servoMin, Math.min(servoMax, pos));
        double t = (pos - servoMin) / (servoMax - servoMin);
        return armMinDeg + t * (armMaxDeg - armMinDeg);
    }

    /**
     * Convert degrees to servo position (0-1).
     */
    public double degToServoPos(double degrees) {
        double t = (degrees - armMinDeg) / (armMaxDeg - armMinDeg);
        t = Math.max(0.0, Math.min(1.0, t));
        return servoMin + t * (servoMax - servoMin);
    }

    /**
     * Check if arm servo is available.
     */
    public boolean isAvailable() {
        return hasArmServo;
    }

    /**
     * Report hardware status.
     */
    public void reportStatus() {
        telemetry.addData("armServo", hasArmServo ? "OK" : "MISSING");
    }

    /**
     * Report current angle telemetry.
     */
    public void reportTelemetry(String presetLabel) {
        if (hasArmServo) {
            double deg = getAngle();
            if (presetLabel != null && !presetLabel.isEmpty()) {
                telemetry.addData("Arm Angle", "%.1f° (%s)", deg, presetLabel);
            } else {
                telemetry.addData("Arm Angle", "%.1f°", deg);
            }
        } else {
            telemetry.addData("Arm Servo", "MISSING");
        }
    }

    /**
     * Preset configuration holder.
     */
    public static class Preset {
        public final String name;
        public final double angleDeg;
        public final double flywheelRPM;

        public Preset(String name, double angleDeg, double flywheelRPM) {
            this.name = name;
            this.angleDeg = angleDeg;
            this.flywheelRPM = flywheelRPM;
        }
    }

    // Common presets
    public static final Preset PRESET_RT = new Preset("RT: 150°/2400 RPM", 150.0, 2400.0);
    public static final Preset PRESET_LT = new Preset("LT: 180°/2200 RPM", 180.0, 2200.0);
    public static final Preset PRESET_X = new Preset("X: 120°/3150 RPM", 120.0, 3150.0);
}
