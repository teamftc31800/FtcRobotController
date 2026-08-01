package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.mechanisms.CameraSettings;
import org.firstinspires.ftc.teamcode.mechanisms.FieldConfig;
import org.firstinspires.ftc.teamcode.mechanisms.PinpointOdometry;
import org.firstinspires.ftc.teamcode.mechanisms.RobotLocalizer;
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
    private RobotLocalizer localizer;

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

    // Camera exposure — adjustable via GP1 bumpers
    private long currentExposure = CameraSettings.exposure;
    private int  currentGain    = CameraSettings.gain;
    private static final long EXPOSURE_STEP = 1;
    private static final int  GAIN_STEP     = 25;
    private boolean lastGp1LB = false;
    private boolean lastGp1RB = false;
    private boolean lastGp1DpadLeft  = false;
    private boolean lastGp1DpadRight = false;

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
                            exposureControl.setMode(ExposureControl.Mode.Manual);
                            exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                        }
                        GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                        if (gainControl != null) {
                            gainControl.setGain(currentGain);
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
            localizer = new RobotLocalizer(
                    odometry,
                    FieldConfig.intoTheDeep2025(),
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

            boolean rbNow = gamepad1.right_bumper;
            if (rbNow && !lastGp1RB && expCtrl != null) {
                long maxExp = expCtrl.getMaxExposure(TimeUnit.MILLISECONDS);
                currentExposure = Math.min(currentExposure + EXPOSURE_STEP, maxExp);
                expCtrl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                CameraSettings.exposure = currentExposure;
                CameraSettings.save(hardwareMap);
            }
            lastGp1RB = rbNow;

            boolean lbNow = gamepad1.left_bumper;
            if (lbNow && !lastGp1LB && expCtrl != null) {
                long minExp = expCtrl.getMinExposure(TimeUnit.MILLISECONDS);
                currentExposure = Math.max(currentExposure - EXPOSURE_STEP, minExp);
                expCtrl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                CameraSettings.exposure = currentExposure;
                CameraSettings.save(hardwareMap);
            }
            lastGp1LB = lbNow;

            boolean drNow = gamepad1.dpad_right;
            if (drNow && !lastGp1DpadRight && gainCtrl != null) {
                int maxGain = gainCtrl.getMaxGain();
                currentGain = Math.min(currentGain + GAIN_STEP, maxGain);
                gainCtrl.setGain(currentGain);
                CameraSettings.gain = currentGain;
                CameraSettings.save(hardwareMap);
            }
            lastGp1DpadRight = drNow;

            boolean dlNow = gamepad1.dpad_left;
            if (dlNow && !lastGp1DpadLeft && gainCtrl != null) {
                int minGain = gainCtrl.getMinGain();
                currentGain = Math.max(currentGain - GAIN_STEP, minGain);
                gainCtrl.setGain(currentGain);
                CameraSettings.gain = currentGain;
                CameraSettings.save(hardwareMap);
            }
            lastGp1DpadLeft = dlNow;
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
        }

        // Visible tags
        if (!allDetections.isEmpty()) {
            for (AprilTagDetection det : allDetections) {
                if (det.ftcPose != null) {
                    telemetry.addData("Tag " + det.id,
                            "R=%.1f\" B=%.1f° Y=%.1f°",
                            det.ftcPose.range, det.ftcPose.bearing, det.ftcPose.yaw);
                }
            }
        } else {
            telemetry.addData("Tags", "none");
        }

        // Camera settings
        if (visionPortal != null) {
            telemetry.addData("Cam", "%.0f FPS | Exp %dms | Gain %d",
                    visionPortal.getFps(), currentExposure, currentGain);
        } else {
            telemetry.addData("Cam", "DOWN");
        }

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("GP1: Sticks=drive | LB/RB=exp | DL/DR=gain");
        telemetry.addLine("GP2: D-pad=offsets | A=reset pose");

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

    private DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); }
        catch (Exception e) { return null; }
    }
}
