package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.mechanisms.CameraSettings;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Minimal TeleOp for isolating camera stability from other robot hardware.
 *
 * Includes ONLY:
 *   - Mecanum drive (gamepad1 left stick = translate, right stick x = rotate)
 *   - AprilTag detection (tag ID 24, TAG_36h11 family)
 *   - Camera state + FPS + pose telemetry
 *
 * Use this to determine if camera disconnects are caused by electrical noise /
 * CPU load from the flywheel, feeder, intake, etc. — by running with those
 * components absent. If the camera is stable here but not in SyncFeeder,
 * the other hardware is the cause.
 */
@TeleOp(name = "CamTest (Drive+Camera)", group = "Drive")
public class TeamBotTeleop_CamTest extends OpMode {

    // AprilTag target (RED goal tag)
    private static final int TARGET_TAG_ID = 24;

    // --- Drivetrain ---
    private DcMotor frontLeft  = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft   = null;
    private DcMotor backRight  = null;

    private boolean hasFrontLeft  = false;
    private boolean hasFrontRight = false;
    private boolean hasBackLeft   = false;
    private boolean hasBackRight  = false;

    // --- Camera ---
    private AprilTagProcessor aprilTag    = null;
    private VisionPortal      visionPortal = null;

    // Latest detection results (updated each loop)
    private boolean tagDetected  = false;
    private double  tagDistance  = 0.0;  // inches
    private double  tagBearing   = 0.0;  // degrees: + right, - left
    private double  tagYaw       = 0.0;  // degrees

    // -------------------------------------------------------------------------
    @Override
    public void init() {
        // Load saved camera settings (survives reboot)
        CameraSettings.load(hardwareMap);

        // --- Drivetrain motors ---
        frontLeft  = safeGetMotor("left_front_drive");
        frontRight = safeGetMotor("right_front_drive");
        backLeft   = safeGetMotor("left_back_drive");
        backRight  = safeGetMotor("right_back_drive");

        hasFrontLeft  = (frontLeft  != null);
        hasFrontRight = (frontRight != null);
        hasBackLeft   = (backLeft   != null);
        hasBackRight  = (backRight  != null);

        // Directions (left side reversed for standard mecanum layout)
        if (hasFrontLeft)  frontLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasBackLeft)   backLeft.setDirection(DcMotor.Direction.REVERSE);
        if (hasFrontRight) frontRight.setDirection(DcMotor.Direction.FORWARD);
        if (hasBackRight)  backRight.setDirection(DcMotor.Direction.FORWARD);

        // Encoder mode + brake
        if (hasFrontLeft)  frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hasFrontRight) frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hasBackLeft)   backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (hasBackRight)  backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (hasFrontLeft)  frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasFrontRight) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackLeft)   backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (hasBackRight)  backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --- Camera (with retry — up to 3 attempts after crash/reboot) ---
        for (int camAttempt = 1; camAttempt <= 3; camAttempt++) {
            try {
                // Wait for USB stack to release — longer delay on retries to let hardware recover
                Thread.sleep(camAttempt == 1 ? 1000 : 2000);

                // AprilTag processor — all visual overlays off, single thread for minimum CPU usage.
                aprilTag = new AprilTagProcessor.Builder()
                        .setDrawAxes(false)
                        .setDrawCubeProjection(false)
                        .setDrawTagOutline(false)
                        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                        .setNumThreads(1)
                        .build();

                // VisionPortal — no live view, MJPEG, C270 calibrated resolution
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .setCameraResolution(new android.util.Size(640, 480))
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .enableLiveView(false)
                        .setAutoStopLiveView(true)
                        .build();

                // Wait up to 3 seconds for camera to reach STREAMING state
                long timeout = System.currentTimeMillis() + 3000;
                while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                    if (System.currentTimeMillis() > timeout) {
                        visionPortal.close();
                        visionPortal = null;
                        break;
                    }
                    telemetry.addData("Camera", "Attempt %d/3 — Waiting...", camAttempt);
                    telemetry.update();
                    try {
                        Thread.sleep(50);
                    } catch (InterruptedException ex) {
                        Thread.currentThread().interrupt();
                        break;
                    }
                }

                // Success — apply exposure settings and stop retrying
                if (visionPortal != null) {
                    try {
                        ExposureControl ec = visionPortal.getCameraControl(ExposureControl.class);
                        if (ec != null) {
                            ec.setMode(ExposureControl.Mode.Manual);
                            ec.setExposure(CameraSettings.exposure, TimeUnit.MILLISECONDS);
                        }
                        GainControl gc = visionPortal.getCameraControl(GainControl.class);
                        if (gc != null) {
                            gc.setGain(CameraSettings.gain);
                        }
                    } catch (Exception ex) {
                        // Camera controls not supported — continue without them
                    }
                    telemetry.addLine("Camera ready (attempt " + camAttempt + ")");
                    break;  // Connected — exit retry loop
                }

                // This attempt failed — log and retry
                telemetry.addData("Camera", "Attempt %d/3 failed — retrying...", camAttempt);
                telemetry.update();

            } catch (Exception e) {
                // Clean up failed portal before retrying
                if (visionPortal != null) {
                    try { visionPortal.close(); } catch (Exception ex) { /* ignore */ }
                    visionPortal = null;
                }
                if (camAttempt < 3) {
                    telemetry.addData("Camera", "Attempt %d/3 error — retrying...", camAttempt);
                    telemetry.update();
                }
            }
        }

        if (visionPortal == null) {
            telemetry.addLine("WARNING: Camera failed after 3 attempts — running without camera");
        }

        telemetry.update();
    }

    // -------------------------------------------------------------------------
    @Override
    public void loop() {

        // --- AprilTag Detection ---
        tagDetected = false;

        if (visionPortal != null && aprilTag != null
                && visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {

            List<AprilTagDetection> detections = aprilTag.getDetections();
            for (AprilTagDetection d : detections) {
                if (d.id == TARGET_TAG_ID && d.ftcPose != null) {
                    tagDetected = true;
                    tagDistance = d.ftcPose.range;    // inches from camera to tag center
                    tagBearing  = d.ftcPose.bearing;  // degrees: + right of center, - left
                    tagYaw      = d.ftcPose.yaw;      // degrees: tag rotation relative to camera
                    break;
                }
            }
        }

        // --- Mecanum Drive ---
        double y  = -gamepad1.left_stick_y;   // forward = +1
        double x  =  gamepad1.left_stick_x;   // strafe right = +1
        double rx =  gamepad1.right_stick_x;  // rotate right = +1

        double fl = y + x + rx;
        double bl = y - x + rx;
        double fr = y - x - rx;
        double br = y + x - rx;

        // Normalize so no value exceeds 1.0
        double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        fl /= max;
        bl /= max;
        fr /= max;
        br /= max;

        if (hasFrontLeft)  frontLeft.setPower(fl);
        if (hasBackLeft)   backLeft.setPower(bl);
        if (hasFrontRight) frontRight.setPower(fr);
        if (hasBackRight)  backRight.setPower(br);

        // --- Telemetry ---
        telemetry.addLine("== CAMERA ==");
        if (visionPortal != null) {
            telemetry.addData("Status",   "Connected");
            telemetry.addData("FPS",      "%.1f", visionPortal.getFps());
            telemetry.addData("CamState", visionPortal.getCameraState().toString());
        } else {
            telemetry.addData("Status",   "NOT CONNECTED");
        }

        telemetry.addLine("== APRILTAG (ID " + TARGET_TAG_ID + ") ==");
        if (tagDetected) {
            telemetry.addData("Tag",      "FOUND");
            telemetry.addData("Distance", "%.1f\"", tagDistance);
            telemetry.addData("Bearing",  "%.1f deg", tagBearing);
            telemetry.addData("Yaw",      "%.1f deg", tagYaw);
        } else {
            telemetry.addData("Tag", "NOT FOUND");
        }

        telemetry.update();
    }

    // -------------------------------------------------------------------------
    @Override
    public void stop() {
        // Stop drive motors
        if (hasFrontLeft)  frontLeft.setPower(0);
        if (hasFrontRight) frontRight.setPower(0);
        if (hasBackLeft)   backLeft.setPower(0);
        if (hasBackRight)  backRight.setPower(0);

        // Release camera cleanly so USB stack is free for the next OpMode
        if (visionPortal != null) {
            try {
                visionPortal.stopStreaming();
                visionPortal.close();
            } catch (Exception e) {
                // Ignore — camera may already be in a bad state
            }
            visionPortal = null;
        }
    }

    // -------------------------------------------------------------------------
    private DcMotor safeGetMotor(String name) {
        try {
            return hardwareMap.get(DcMotor.class, name);
        } catch (Exception e) {
            return null;
        }
    }
}
