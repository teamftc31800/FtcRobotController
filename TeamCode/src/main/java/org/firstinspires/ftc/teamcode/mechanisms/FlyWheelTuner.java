package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp(name = "FlyWheelTuner", group = "Tuning")
public class FlyWheelTuner extends OpMode {
    // Matches SyncFeeder exactly: launcher=FORWARD, launcher2=REVERSE
    private DcMotorEx flywheelMotor;   // launcher
    private DcMotorEx flywheelMotor2;  // launcher2 (optional — same PIDF applied if present)
    private boolean hasMotor2 = false;

    // RPM targets — Y button cycles through these
    private double[] rpmTargets = {2000, 2700, 2850};
    private int rpmIndex = 0;
    private double curTargetVelocity = 2000;

    private static final double FLYWHEEL_TPR = 28.0;

    // PIDF coefficients — adjust these via D-Pad during tuning
    double F = 13.8;
    double P = 20.9;

    // Step size for D-Pad adjustments — B button cycles through these
    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    // Edge-detect flags for button presses
    private boolean lastY = false;
    private boolean lastB = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void init() {
        // launcher — FORWARD, matches SyncFeeder
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.FORWARD);  // matches SyncFeeder
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F));

        // launcher2 — REVERSE, matches SyncFeeder (optional, graceful skip if missing)
        try {
            flywheelMotor2 = hardwareMap.get(DcMotorEx.class, "launcher2");
            flywheelMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            flywheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            flywheelMotor2.setDirection(DcMotorSimple.Direction.REVERSE);  // matches SyncFeeder
            flywheelMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(P, 0, 0, F));
            hasMotor2 = true;
        } catch (Exception e) {
            telemetry.addLine("⚠️ launcher2 not found — tuning with 1 motor only");
        }

        telemetry.addLine("FlyWheelTuner ready (matches SyncFeeder setup)");
        telemetry.addLine("D-Pad L/R = adjust F | D-Pad U/D = adjust P");
        telemetry.addLine("B = cycle step size | Y = cycle RPM target");
        telemetry.update();
    }

    @Override
    public void loop() {
        // --- Y button: cycle RPM target ---
        boolean yNow = gamepad1.y;
        if (yNow && !lastY) {
            rpmIndex = (rpmIndex + 1) % rpmTargets.length;
            curTargetVelocity = rpmTargets[rpmIndex];
        }
        lastY = yNow;

        // --- B button: cycle step size ---
        boolean bNow = gamepad1.b;
        if (bNow && !lastB) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        lastB = bNow;

        // --- D-Pad Left/Right: adjust F ---
        boolean dpadLeftNow = gamepad1.dpad_left;
        boolean dpadRightNow = gamepad1.dpad_right;
        if (dpadLeftNow && !lastDpadLeft) {
            F += stepSizes[stepIndex];
        }
        if (dpadRightNow && !lastDpadRight) {
            F -= stepSizes[stepIndex];
        }
        lastDpadLeft = dpadLeftNow;
        lastDpadRight = dpadRightNow;

        // --- D-Pad Up/Down: adjust P ---
        boolean dpadUpNow = gamepad1.dpad_up;
        boolean dpadDownNow = gamepad1.dpad_down;
        if (dpadUpNow && !lastDpadUp) {
            P += stepSizes[stepIndex];
        }
        if (dpadDownNow && !lastDpadDown) {
            P -= stepSizes[stepIndex];
        }
        lastDpadUp = dpadUpNow;
        lastDpadDown = dpadDownNow;

        // Apply updated PIDF to both motors
        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        flywheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        if (hasMotor2) {
            flywheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        }

        // Set velocity on both motors
        double targetTPS = rpmToTicksPerSec(curTargetVelocity, FLYWHEEL_TPR);
        flywheelMotor.setVelocity(targetTPS);
        if (hasMotor2) {
            flywheelMotor2.setVelocity(targetTPS);
        }

        // Read actual velocity from launcher (primary motor)
        double curTPS = flywheelMotor.getVelocity();
        double curVelocity = ticksPerSecToRPM(curTPS, FLYWHEEL_TPR);
        double error = curTargetVelocity - curVelocity;

        // Telemetry
        telemetry.addData("Target RPM", "%.0f", curTargetVelocity);
        telemetry.addData("Current RPM", "%.1f", curVelocity);
        telemetry.addData("Error", "%.1f (goal: near 0)", error);
        telemetry.addData("Motors", hasMotor2 ? "launcher + launcher2" : "launcher only");
        telemetry.addLine("-----------------------------");
        telemetry.addData("F value", "%.4f  (D-Pad L+/R-)", F);
        telemetry.addData("P value", "%.4f  (D-Pad U+/D-)", P);
        telemetry.addData("Step Size", "%.4f  (B to cycle)", stepSizes[stepIndex]);
        telemetry.update();
    }

    @Override
    public void stop() {
        flywheelMotor.setVelocity(0);
        if (hasMotor2) {
            flywheelMotor2.setVelocity(0);
        }
    }

    private double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        double ticksPerMinute = rpm * ticksPerRev;
        return ticksPerMinute / 60.0;
    }

    private double ticksPerSecToRPM(double ticksPerSec, double ticksPerRev) {
        double ticksPerMin = ticksPerSec * 60.0;
        return ticksPerMin / ticksPerRev;
    }
}
