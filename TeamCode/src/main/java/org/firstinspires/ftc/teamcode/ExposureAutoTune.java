package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
 * Auto-tunes camera exposure AND gain to detect AprilTags at any brightness.
 *
 * STRATEGY — starts dark, adds brightness in the cheapest way:
 *   1. Begin at minimum exposure + minimum gain (darkest possible image)
 *   2. Increase GAIN first (adds brightness without motion blur)
 *   3. Once gain is maxed, increase EXPOSURE by 1ms and reset gain to min
 *   4. Repeat until a tag is detected
 *
 * This finds the lowest exposure+gain combo that sees the tag, which means:
 *   - Bright venue → low exposure, low gain  (no blur, no noise)
 *   - Dim venue    → low exposure, high gain  (no blur, some noise)
 *   - Very dim     → higher exposure + gain   (last resort)
 *
 * HOW TO USE:
 *   1. Point the camera at an AprilTag at your venue
 *   2. Press PLAY — the program sweeps automatically
 *   3. When "TAG FOUND" appears, the values are saved
 *   4. All other programs will use the saved exposure+gain
 *
 * Saved values persist across robot reboots.
 */
@TeleOp(name="ExposureAutoTune", group="Testing")
public class ExposureAutoTune extends OpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private ExposureControl exposureControl;
    private GainControl gainControl;

    // Exposure range (ms)
    private long currentExposure;
    private long minExposure;
    private long maxExposure;

    // Gain range
    private int currentGain;
    private int minGain;
    private int maxGain;

    // How much to increase gain each step
    private static final int GAIN_STEP = 25;

    // State
    private boolean found = false;
    private long foundExposure = 0;
    private int  foundGain = 0;
    private boolean failed = false;

    // Wait frames after each setting change for the camera to adjust
    private static final int FRAMES_TO_SETTLE = 10;
    private int settleCounter = 0;

    // Track total combinations tested for progress display
    private int stepsTested = 0;
    private int totalSteps  = 0;

    @Override
    public void init() {
        // --- Webcam with retry ---
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
                    exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                    gainControl = visionPortal.getCameraControl(GainControl.class);

                    // Get exposure range
                    if (exposureControl != null) {
                        exposureControl.setMode(ExposureControl.Mode.Manual);
                        minExposure = exposureControl.getMinExposure(TimeUnit.MILLISECONDS);
                        maxExposure = exposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
                        currentExposure = minExposure;
                        exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                    }

                    // Get gain range
                    if (gainControl != null) {
                        minGain = gainControl.getMinGain();
                        maxGain = gainControl.getMaxGain();
                        currentGain = minGain;
                        gainControl.setGain(currentGain);
                    }

                    // Calculate total steps for progress bar
                    int gainStepsPerExposure = ((maxGain - minGain) / GAIN_STEP) + 1;
                    int exposureSteps = (int)(maxExposure - minExposure) + 1;
                    totalSteps = gainStepsPerExposure * exposureSteps;

                    telemetry.addLine("Webcam connected (attempt " + camAttempt + ")");
                    telemetry.addData("Exposure range", "%d - %d ms", minExposure, maxExposure);
                    telemetry.addData("Gain range", "%d - %d", minGain, maxGain);
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
            telemetry.addLine("Webcam failed — cannot auto-tune");
        }

        telemetry.addLine("Point camera at an AprilTag, then press PLAY");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (visionPortal == null || exposureControl == null || gainControl == null) {
            telemetry.addLine("NO CAMERA — cannot tune");
            telemetry.update();
            return;
        }

        // --- Already found ---
        if (found) {
            telemetry.addLine("=== TAG FOUND ===");
            telemetry.addData("Exposure", "%d ms", foundExposure);
            telemetry.addData("Gain", "%d", foundGain);
            telemetry.addLine("");
            telemetry.addLine("Saved — all programs will use these settings");
            telemetry.addLine("Settings survive reboot");
            telemetry.update();
            return;
        }

        // --- All combos exhausted ---
        if (failed) {
            telemetry.addLine("=== NO TAG FOUND ===");
            telemetry.addData("Scanned", "Exp %d-%d ms x Gain %d-%d",
                    minExposure, maxExposure, minGain, maxGain);
            telemetry.addLine("Check: is a tag visible to the camera?");
            telemetry.update();
            return;
        }

        // --- Settling after a settings change ---
        if (settleCounter < FRAMES_TO_SETTLE) {
            settleCounter++;
            telemetry.addLine("=== EXPOSURE + GAIN AUTO-TUNE ===");
            telemetry.addData("Testing", "Exp %d ms | Gain %d  (settling %d/%d)",
                    currentExposure, currentGain, settleCounter, FRAMES_TO_SETTLE);
            showProgress();
            telemetry.update();
            return;
        }

        // --- Check for AprilTag ---
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            List<AprilTagDetection> detections = aprilTag.getDetections();

            for (AprilTagDetection det : detections) {
                if (det.ftcPose != null) {
                    // Found a tag — save both exposure and gain
                    found = true;
                    foundExposure = currentExposure;
                    foundGain = currentGain;

                    CameraSettings.exposure = foundExposure;
                    CameraSettings.gain = foundGain;
                    CameraSettings.save(hardwareMap);

                    telemetry.addLine("=== TAG FOUND ===");
                    telemetry.addData("Exposure", "%d ms", foundExposure);
                    telemetry.addData("Gain", "%d", foundGain);
                    telemetry.addData("Tag", "ID %d at %.1f inches", det.id, det.ftcPose.range);
                    telemetry.addLine("");
                    telemetry.addLine("Saved — all programs will use these settings");
                    telemetry.update();
                    return;
                }
            }
        }

        // --- No tag found — advance to next setting ---
        stepsTested++;

        // Strategy: increase gain first (cheaper), then bump exposure
        if (currentGain + GAIN_STEP <= maxGain) {
            // Still room to increase gain at this exposure level
            currentGain += GAIN_STEP;
            gainControl.setGain(currentGain);
        } else if (currentExposure < maxExposure) {
            // Gain maxed — increase exposure by 1ms and reset gain to minimum
            currentExposure++;
            currentGain = minGain;
            exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
            gainControl.setGain(currentGain);
        } else {
            // Both maxed — nothing left to try
            failed = true;
        }

        settleCounter = 0;  // reset for new settings

        telemetry.addLine("=== EXPOSURE + GAIN AUTO-TUNE ===");
        telemetry.addData("Testing", "Exp %d ms | Gain %d", currentExposure, currentGain);
        telemetry.addData("Status", "Searching...");
        showProgress();
        telemetry.update();
    }

    /** Show a progress indicator so you know how far along the sweep is. */
    private void showProgress() {
        if (totalSteps > 0) {
            int percent = (stepsTested * 100) / totalSteps;
            telemetry.addData("Progress", "%d%% (%d/%d combos)", percent, stepsTested, totalSteps);
        }
        telemetry.addData("Exp range", "%d - %d ms", minExposure, maxExposure);
        telemetry.addData("Gain range", "%d - %d", minGain, maxGain);
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
}
