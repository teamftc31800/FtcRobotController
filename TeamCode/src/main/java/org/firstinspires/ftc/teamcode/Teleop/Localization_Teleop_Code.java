
// Seperate Top Level OpMode File





// IGNORE CODE BELOW UNLESS NECESSARY FOR THIS SEASONS






























package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.mechanisms.CameraSettings;
import org.firstinspires.ftc.teamcode.mechanisms.FieldConfig;
import org.firstinspires.ftc.teamcode.mechanisms.PinpointOdometry;
import org.firstinspires.ftc.teamcode.mechanisms.RobotLocalizerEDITEDBYCLAUDE;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Localization test TeleOp — drive + localizer + live offset tuning.
 * All mechanisms (flywheel, intake, feeder, arm, RGB) are disabled.
 *
 * OFFSET TUNING:
 *   1. Spin robot in place using GP1 right stick
 *   2. Watch "Odo Raw" X and Y — they should stay near 0 during pure rotation
 *   3. Use GP2 D-pad to adjust offsets until X/Y stop drifting
 *   4. Note the final xOffset/yOffset values from telemetry
 *   5. Hardcode those values into your competition TeleOp
 *
 * GP2 A = reset pose to (0,0,0) — use before each spin test
 */
@TeleOp(name="Localization_Teleop_Code", group="Testing")
public class Localization_Teleop_Code extends OpMode {

    // Drivetrain
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    // Camera + localizer
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private PinpointOdometry odometry;
    private RobotLocalizerEDITEDBYCLAUDE localizer;

    // Odometry pod offsets — tune these with GP2 D-pad
    private double xOffset = 0.0;    // lateral offset of forward pod (left=+, right=-)
    private double yOffset = -2.0;   // longitudinal offset of strafe pod (fwd=+, back=-)
    private static final double OFFSET_STEP = 0.5;  // inches per click

    // Edge-detect for GP2 offset tuning
    private boolean lastGp2DpadUp    = false;
    private boolean lastGp2DpadDown  = false;
    private boolean lastGp2DpadLeft  = false;
    private boolean lastGp2DpadRight = false;
    private boolean lastGp2A         = false;
    private boolean lastGp2Y         = false;
    private boolean snapTestMode     = false;  // GP2 Y toggles this

    // Camera exposure — adjustable via GP1 bumpers
    private long currentExposure = CameraSettings.exposure;
    private int  currentGain    = CameraSettings.gain;
    private static final long EXPOSURE_STEP = 1;
    private static final int  GAIN_STEP     = 25;
    private boolean lastGp1LB = false;
    private boolean lastGp1RB = false;
    private boolean lastGp1DpadLeft  = false;
    private boolean lastGp1DpadRight = false;
    private String lastCameraChange = "none yet";

    @Override
    public void init() {
        // Load saved camera settings (survives reboot)
        CameraSettings.load(hardwareMap);
        currentExposure = CameraSettings.exposure;
        currentGain = CameraSettings.gain;

        // --- Drivetrain ---
        frontLeft  = getMotor("left_front_drive");
        frontRight = getMotor("right_front_drive");
        backLeft   = getMotor("left_back_drive");
        backRight  = getMotor("right_back_drive");

        if (frontLeft != null)  { frontLeft.setDirection(DcMotor.Direction.REVERSE);  frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
        if (backLeft != null)   { backLeft.setDirection(DcMotor.Direction.REVERSE);   backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
        if (frontRight != null) { frontRight.setDirection(DcMotor.Direction.FORWARD); frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }
        if (backRight != null)  { backRight.setDirection(DcMotor.Direction.FORWARD);  backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); }

        // --- Webcam ---
        for (int camAttempt = 1; camAttempt <= 3; camAttempt++) {
            try {
                Thread.sleep(camAttempt == 1 ? 1000 : 2000);

                aprilTag = new AprilTagProcessor.Builder()
                        .setDrawAxes(true)
                        .setDrawCubeProjection(false)
                        .setDrawTagOutline(true)
                        .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                        .setNumThreads(1)
                        .build();

                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .setCameraResolution(new android.util.Size(640, 480))
                        .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .enableLiveView(false)
                        .setAutoStopLiveView(true)
                        .build();

                long timeout = System.currentTimeMillis() + 3000;
                while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                    if (System.currentTimeMillis() > timeout) {
                        visionPortal.close();
                        visionPortal = null;
                        break;
                    }
                    telemetry.addData("Webcam", "Attempt %d/3 — Waiting...", camAttempt);
                    telemetry.update();
                    try { Thread.sleep(50); } catch (InterruptedException ex) { Thread.currentThread().interrupt(); break; }
                }

                if (visionPortal != null) {
                    try {
                        ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                        if (exposureControl != null) {
                            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                                exposureControl.setMode(ExposureControl.Mode.Manual);
                                Thread.sleep(50);
                            }
                            exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                            Thread.sleep(20);
                        }
                        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                        if (gainControl != null) {
                            gainControl.setGain(currentGain);
                            Thread.sleep(20);
                        }
                    } catch (Exception ex) { /* camera controls not supported */ }
                    telemetry.addLine("Webcam connected (attempt " + camAttempt + ")");
                    break;
                }
            } catch (Exception e) {
                if (visionPortal != null) {
                    try { visionPortal.close(); } catch (Exception ex) { /* ignore */ }
                    visionPortal = null;
                }
            }
        }
        if (visionPortal == null) {
            telemetry.addLine("Webcam failed — running without camera");
        }

        // --- Localizer ---
        try {
            odometry = new PinpointOdometry(
                    hardwareMap,
                    0, 0, 0,
                    xOffset, yOffset,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED
            );
            localizer = new RobotLocalizerEDITEDBYCLAUDE(
                    odometry,
                    FieldConfig.biobuzz2026_2027(),
                    0, 0, 0,
                    0.3
            );
            telemetry.addLine("Localizer ready");
        } catch (Exception e) {
            telemetry.addLine("Pinpoint not found — localizer disabled");
        }

        telemetry.update();
    }

    @Override
    public void loop() {

        // --- 1. DRIVE (mecanum, GP1 sticks) ---
        double y  = -gamepad1.left_stick_y;
        double x  =  gamepad1.left_stick_x;
        double rx =  gamepad1.right_stick_x;

        double flPower = y + x + rx;
        double blPower = y - x + rx;
        double frPower = y - x - rx;
        double brPower = y + x - rx;

        double max = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        if (frontLeft != null)  frontLeft.setPower(flPower / max);
        if (backLeft != null)   backLeft.setPower(blPower / max);
        if (frontRight != null) frontRight.setPower(frPower / max);
        if (backRight != null)  backRight.setPower(brPower / max);

        // --- 2. OFFSET TUNING (GP2 D-pad) ---
        // D-pad UP/DOWN = adjust yOffset (strafe pod forward/back)
        // D-pad LEFT/RIGHT = adjust xOffset (forward pod left/right)
        // A = reset pose to (0,0,0) for clean spin test
        if (odometry != null) {
            boolean upNow    = gamepad2.dpad_up;
            boolean downNow  = gamepad2.dpad_down;
            boolean leftNow  = gamepad2.dpad_left;
            boolean rightNow = gamepad2.dpad_right;
            boolean aNow     = gamepad2.a;

            if (upNow && !lastGp2DpadUp) {
                yOffset += OFFSET_STEP;
                odometry.setOffsets(xOffset, yOffset);
            }
            if (downNow && !lastGp2DpadDown) {
                yOffset -= OFFSET_STEP;
                odometry.setOffsets(xOffset, yOffset);
            }
            if (rightNow && !lastGp2DpadRight) {
                xOffset -= OFFSET_STEP;  // right = negative in GoBilda convention
                odometry.setOffsets(xOffset, yOffset);
            }
            if (leftNow && !lastGp2DpadLeft) {
                xOffset += OFFSET_STEP;  // left = positive in GoBilda convention
                odometry.setOffsets(xOffset, yOffset);
            }
            if (aNow && !lastGp2A) {
                odometry.setPose(0, 0, 0);
                if (localizer != null) localizer.resetPose(0, 0, 0);
            }

            // GP2 Y = toggle "snap test mode": alpha=1.0 and a huge outlier
            // cap, so ONE detection fully snaps the Fused pose to whatever
            // the tag math computes — use this to verify fusion with a
            // deliberately made-up tag coordinate. Toggle off to go back to
            // the smoothed 0.3/24" behavior meant for real driving.
            boolean yNow = gamepad2.y;
            if (yNow && !lastGp2Y && localizer != null) {
                snapTestMode = !snapTestMode;
                if (snapTestMode) {
                    localizer.setCorrectionAlpha(1.0);
                    localizer.setMaxCorrectionInches(100000);
                } else {
                    localizer.setCorrectionAlpha(0.3);
                    localizer.setMaxCorrectionInches(24);
                }
            }
            lastGp2Y = yNow;

            lastGp2DpadUp    = upNow;
            lastGp2DpadDown  = downNow;
            lastGp2DpadLeft  = leftNow;
            lastGp2DpadRight = rightNow;
            lastGp2A         = aNow;
        }

        // --- 3. APRILTAG DETECTION ---
        List<AprilTagDetection> allDetections = new ArrayList<>();
        if (visionPortal != null && aprilTag != null) {
            VisionPortal.CameraState camState = visionPortal.getCameraState();
            if (camState == VisionPortal.CameraState.STREAMING) {
                allDetections = aprilTag.getDetections();
            }
        }

        // --- 4. LOCALIZER UPDATE ---
        if (localizer != null) {
            localizer.update(allDetections);
        }

        // --- 5. CAMERA EXPOSURE/GAIN (GP1 bumpers + D-pad L/R) ---
        if (visionPortal != null) {
            ExposureControl expCtrl = visionPortal.getCameraControl(ExposureControl.class);
            GainControl gainCtrl = visionPortal.getCameraControl(GainControl.class);

            boolean expUpNow   = gamepad1.right_bumper || gamepad1.y;
            boolean expDownNow = gamepad1.left_bumper  || gamepad1.a;
            boolean gainUpNow   = gamepad1.dpad_right || gamepad1.b;
            boolean gainDownNow = gamepad1.dpad_left  || gamepad1.x;

            String pressed = null;
            if (expUpNow && !lastGp1RB) {
                if (expCtrl != null) currentExposure = Math.min(currentExposure + EXPOSURE_STEP, expCtrl.getMaxExposure(TimeUnit.MILLISECONDS));
                pressed = "Exposure UP";
            }
            lastGp1RB = expUpNow;

            if (expDownNow && !lastGp1LB) {
                if (expCtrl != null) currentExposure = Math.max(currentExposure - EXPOSURE_STEP, expCtrl.getMinExposure(TimeUnit.MILLISECONDS));
                pressed = "Exposure DOWN";
            }
            lastGp1LB = expDownNow;

            if (gainUpNow && !lastGp1DpadRight) {
                if (gainCtrl != null) currentGain = Math.min(currentGain + GAIN_STEP, gainCtrl.getMaxGain());
                pressed = "Gain UP";
            }
            lastGp1DpadRight = gainUpNow;

            if (gainDownNow && !lastGp1DpadLeft) {
                if (gainCtrl != null) currentGain = Math.max(currentGain - GAIN_STEP, gainCtrl.getMinGain());
                pressed = "Gain DOWN";
            }
            lastGp1DpadLeft = gainDownNow;

            if (pressed != null) {
                applyCameraSettings(expCtrl, gainCtrl, pressed);
            }
        }

        // --- 6. TELEMETRY ---
        telemetry.addLine("=== LOCALIZATION TEST ===");

        // Pod offsets (the values you're tuning)
        telemetry.addData("Offsets", "xOff=%.1f  yOff=%.1f  (GP2 D-pad to tune)",
                xOffset, yOffset);

        // Raw odometry — watch this during spin test
        if (odometry != null) {
            telemetry.addData("Odo Raw", "X=%.1f  Y=%.1f  H=%.1f°",
                    odometry.getX(), odometry.getY(), odometry.getHeadingDeg());
        }


        // Localizer fused position
        if (localizer != null) {
            telemetry.addData("Fused", "X=%.1f  Y=%.1f  H=%.1f°%s",
                    localizer.getX(), localizer.getY(), localizer.getHeadingDeg(),
                    localizer.hadTagCorrection() ? " [TAG]" : "");

            telemetry.addData("Snap test mode", snapTestMode
                    ? "ON (alpha=1.0, cap=huge) — GP2 Y to turn off"
                    : "off (alpha=0.3, cap=24in) — GP2 Y to snap-test");

            // Raw, unblended pose the math computed from the tag(s) this loop —
            // compare this directly against your hand-calculated expected value
            if (localizer.hadUsableTagThisUpdate()) {
                telemetry.addData("Tag-only pose (raw)", "X=%.1f  Y=%.1f  H=%.1f°",
                        localizer.getLastTagOnlyX(), localizer.getLastTagOnlyY(), localizer.getLastTagOnlyHeadingDeg());
                if (localizer.wasLastTagRejectedAsOutlier()) {
                    telemetry.addLine("  ^ REJECTED as outlier (>"
                            + localizer.getMaxCorrectionInches() + "in from Fused) — not blended in. Turn on Snap test mode to bypass.");
                }
            }
        }

        // Visible tags
        if (!allDetections.isEmpty()) {
            for (AprilTagDetection det : allDetections) {
                if (det.ftcPose != null) {
                    telemetry.addData("Tag " + det.id,
                            "R=%.1f\" B=%.1f° Y=%.1f°",
                            det.ftcPose.range, det.ftcPose.bearing, det.ftcPose.yaw);
                } else {
                    telemetry.addData("Tag " + det.id, "detected, no pose (ID not in tag library?)");
                }
            }
        } else {
            telemetry.addData("Tags", "none");
        }

        // Camera settings
        if (visionPortal != null) {
            telemetry.addData("Cam", "%.0f FPS | Exp %dms | Gain %d (requested)",
                    visionPortal.getFps(), currentExposure, currentGain);
            telemetry.addData("Last cam change", lastCameraChange);
            try {
                ExposureControl ec = visionPortal.getCameraControl(ExposureControl.class);
                GainControl gc = visionPortal.getCameraControl(GainControl.class);
                telemetry.addData("Cam actual", "mode=%s | Exp %dms | Gain %d",
                        ec != null ? ec.getMode() : "n/a",
                        ec != null ? ec.getExposure(TimeUnit.MILLISECONDS) : -1,
                        gc != null ? gc.getGain() : -1);
            } catch (Exception e) {
                telemetry.addData("Cam actual", "read failed: %s", e.getMessage());
            }
        } else {
            telemetry.addData("Cam", "DOWN");
        }

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("GP1: Sticks=drive | Exp: RB/Y=up LB/A=down | Gain: DR/B=up DL/X=down");
        telemetry.addLine("GP2: D-pad=offsets | A=reset pose | Y=toggle snap test mode");

        telemetry.update();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            try {
                visionPortal.stopStreaming();
                visionPortal.close();
            } catch (Exception e) { /* ignore */ }
            visionPortal = null;
        }
    }

    /** Forces Manual mode, applies the requested exposure/gain, reads back what the camera took, and reports it. */
    private void applyCameraSettings(ExposureControl expCtrl, GainControl gainCtrl, String reason) {
        StringBuilder msg = new StringBuilder(reason).append(": ");
        try {
            if (expCtrl == null) {
                msg.append("ExposureControl NULL (unsupported). ");
            } else {
                if (expCtrl.getMode() != ExposureControl.Mode.Manual) {
                    expCtrl.setMode(ExposureControl.Mode.Manual);
                    Thread.sleep(50);
                }
                expCtrl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                Thread.sleep(20);
                msg.append(String.format("Exp req %dms -> actual %dms (%s). ",
                        currentExposure, expCtrl.getExposure(TimeUnit.MILLISECONDS), expCtrl.getMode()));
            }
            if (gainCtrl == null) {
                msg.append("GainControl NULL (unsupported).");
            } else {
                gainCtrl.setGain(currentGain);
                Thread.sleep(20);
                msg.append(String.format("Gain req %d -> actual %d.", currentGain, gainCtrl.getGain()));
            }
        } catch (Exception e) {
            msg.append("ERROR ").append(e.getMessage());
        }
        CameraSettings.exposure = currentExposure;
        CameraSettings.gain = currentGain;
        CameraSettings.save(hardwareMap);
        lastCameraChange = msg.toString();
        RobotLog.ii("CamTune", lastCameraChange);
    }

    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); }
        catch (Exception e) { return null; }
    }
}
