package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Mecanum drivetrain module.
 * Handles motor initialization, direction setup, and drive calculations.
 */
public class Drivetrain {

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    private boolean hasFrontLeft = false;
    private boolean hasFrontRight = false;
    private boolean hasBackLeft = false;
    private boolean hasBackRight = false;

    private Telemetry telemetry;

    /**
     * Initialize the drivetrain with default motor names.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        init(hardwareMap, telemetry,
             "left_front_drive", "right_front_drive",
             "left_back_drive", "right_back_drive");
    }

    /**
     * Initialize the drivetrain with custom motor names.
     */
    public void init(HardwareMap hardwareMap, Telemetry telemetry,
                     String flName, String frName, String blName, String brName) {
        this.telemetry = telemetry;

        frontLeft = getMotor(hardwareMap, flName);
        frontRight = getMotor(hardwareMap, frName);
        backLeft = getMotor(hardwareMap, blName);
        backRight = getMotor(hardwareMap, brName);

        hasFrontLeft = (frontLeft != null);
        hasFrontRight = (frontRight != null);
        hasBackLeft = (backLeft != null);
        hasBackRight = (backRight != null);

        // Set directions (left side reversed for mecanum)
        if (hasFrontLeft) frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasBackLeft) backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasFrontRight) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (hasBackRight) backRight.setDirection(DcMotor.Direction.FORWARD);

        // Set brake mode
        if (hasFrontLeft) frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasFrontRight) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackLeft) backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackRight) backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Drive using mecanum calculations.
     * @param y Forward/backward (-1 to 1)
     * @param x Strafe left/right (-1 to 1)
     * @param rx Rotation (-1 to 1)
     */
    public void drive(double y, double x, double rx) {
        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        // Normalize
        double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        flPower /= max;
        blPower /= max;
        frPower /= max;
        brPower /= max;

        setPowers(flPower, frPower, blPower, brPower);
    }

    /**
     * Drive using gamepad stick inputs directly.
     */
    public void driveWithGamepad(double leftStickY, double leftStickX, double rightStickX) {
        double y = -leftStickY;  // forward = +1
        double x = leftStickX;   // strafe right = +1
        double rx = rightStickX; // rotate right = +1
        drive(y, x, rx);
    }

    /**
     * Set individual motor powers.
     */
    public void setPowers(double fl, double fr, double bl, double br) {
        if (hasFrontLeft) frontLeft.setPower(fl);
        if (hasFrontRight) frontRight.setPower(fr);
        if (hasBackLeft) backLeft.setPower(bl);
        if (hasBackRight) backRight.setPower(br);
    }

    /**
     * Stop all motors.
     */
    public void stop() {
        setPowers(0, 0, 0, 0);
    }

    /**
     * Report hardware status to telemetry.
     */
    public void reportStatus() {
        telemetry.addData("frontLeft", hasFrontLeft ? "OK" : "MISSING");
        telemetry.addData("frontRight", hasFrontRight ? "OK" : "MISSING");
        telemetry.addData("backLeft", hasBackLeft ? "OK" : "MISSING");
        telemetry.addData("backRight", hasBackRight ? "OK" : "MISSING");
    }

    /**
     * Check if all motors are available.
     */
    public boolean isFullyOperational() {
        return hasFrontLeft && hasFrontRight && hasBackLeft && hasBackRight;
    }

    private DcMotor getMotor(HardwareMap hardwareMap, String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }
}
