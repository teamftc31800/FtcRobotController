package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
 * Logitech C270 HD Webcam AprilTag Tracking for FTC Into The Deep 2025-2026
 *
 * This OpMode uses a Logitech C270 HD Webcam (720p) to detect and track AprilTags.
 * It provides distance and angle information for autonomous navigation and alignment.
 *
 * Hardware Requirements:
 * - Logitech C270 HD Webcam connected to USB port on REV Control Hub
 * - Configure device name as "Webcam 1" in the robot configuration
 *
 * AprilTag IDs for Into The Deep (2025-2026):
 * - Tags 11-20: Field elements and scoring positions
 * - Check game manual for specific tag assignments
 *
 * Distance and Angle Information:
 * - Range: Distance from camera to tag (in inches)
 * - Bearing: Horizontal angle to tag (degrees, + = right, - = left)
 * - Yaw: Tag's rotation around vertical axis (degrees)
 * - Pitch: Tag's tilt up/down (degrees)
 * - Roll: Tag's rotation around horizontal axis (degrees)
 */
@TeleOp(name="Webcam AprilTag Tracking", group="Vision")
public class WebcamAprilTagTracking extends OpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Target tag configuration
    private int targetTagId = -1;  // -1 = track any tag, or set to specific tag ID

    // Last known detection data
    private double lastKnownRange = 0.0;
    private double lastKnownBearing = 0.0;
    private double lastKnownYaw = 0.0;
    private long lastDetectionTime = 0;
    private static final long DETECTION_TIMEOUT_MS = 500;  // Consider tag "lost" after 500ms

    // Edge detection for controls
    private boolean lastDpadUp = false;
    private boolean lastAButton = false;
    private boolean lastBButton = false;

    @Override
    public void init() {
        // Load saved camera settings (survives reboot)
        CameraSettings.load(hardwareMap);
        currentExposure = (int) CameraSettings.exposure;
        currentGain = CameraSettings.gain;

        // Create the AprilTag processor with optimized settings for varying lighting
        aprilTag = new AprilTagProcessor.Builder()
                // Visual feedback (can be disabled to improve performance)
                .setDrawAxes(true)          // Draw 3D axes on detected tags
                .setDrawCubeProjection(true) // Draw 3D cube projection on tags
                .setDrawTagOutline(true)     // Draw outline around tags

                // Detection quality settings
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)  // TAG_36h11 is more robust than TAG_standard41h12

                // Improved detection parameters
                .setNumThreads(3)           // Use multiple threads for faster processing
                .build();

        // Create the vision portal with optimized camera settings
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)

                // Camera resolution - 640x480 provides optimal balance
                // Options: 640x480, 800x600, 1280x720, 1920x1080
                // Using 640x480 because:
                // 1. FTC SDK has built-in calibration data for C270 at this resolution
                // 2. 4x faster processing than 1280x720
                // 3. Better low-light performance (larger effective pixels)
                // 4. Sufficient FOV for FTC AprilTag detection
                .setCameraResolution(new android.util.Size(640, 480))

                // Stream format - MJPEG typically works better than YUY2
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)

                // Enable/disable live camera view to save processing power
                .enableLiveView(true)

                // Auto-stop streaming to improve performance after detection stabilizes
                .setAutoStopLiveView(false);

        // Add camera calibration if available (will use system calibration if it exists)
        // If no calibration exists, AprilTag detection will still work but distances may be less accurate
        visionPortal = builder.build();

        // Wait for vision portal to be ready
        while (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting for camera to start...");
            telemetry.update();
            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }

        // Apply camera controls for better lighting adaptation
        // Note: These may not work on all webcams
        try {
            // Get exposure control and set to manual mode
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl != null) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
                telemetry.addData("Exposure", "%d ms (manual)", currentExposure);
            }

            // Get gain control and set gain
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            if (gainControl != null) {
                gainControl.setGain(currentGain);
                telemetry.addData("Gain", "%d", currentGain);
            }

        } catch (Exception e) {
            telemetry.addLine("⚠️ Camera controls not supported on this webcam");
        }

        telemetry.addLine("== WEBCAM APRILTAG TRACKING ==");
        telemetry.addData("Webcam", "Logitech C270 HD (720p)");
        telemetry.addData("Status", "Initializing...");
        telemetry.addLine("\n== INSTRUCTIONS ==");
        telemetry.addData("D-pad UP", "Track ANY tag");
        telemetry.addData("A/B", "Adjust target tag ID (+/-)");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        // Allow tag ID selection during init
        boolean dpadUpNow = gamepad1.dpad_up;
        boolean aButtonNow = gamepad1.a;
        boolean bButtonNow = gamepad1.b;

        // D-pad UP: Track any tag
        if (dpadUpNow && !lastDpadUp) {
            targetTagId = -1;
        }

        // A button: Increment tag ID
        if (aButtonNow && !lastAButton && targetTagId < 20) {
            targetTagId++;
        }

        // B button: Decrement tag ID
        if (bButtonNow && !lastBButton && targetTagId > -1) {
            targetTagId--;
        }

        lastDpadUp = dpadUpNow;
        lastAButton = aButtonNow;
        lastBButton = bButtonNow;

        telemetry.addData("Target Tag ID", targetTagId == -1 ? "ANY" : targetTagId);
        telemetry.addData("Camera Status", visionPortal.getCameraState());
        telemetry.addData("Status", "Ready - Press Play to start tracking");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Control: Adjust target tag during operation
        boolean dpadUpNow = gamepad1.dpad_up;
        boolean aButtonNow = gamepad1.a;
        boolean bButtonNow = gamepad1.b;
        boolean xButtonNow = gamepad1.x;
        boolean yButtonNow = gamepad1.y;

        // D-pad UP: Track any tag
        if (dpadUpNow && !lastDpadUp) {
            targetTagId = -1;
        }

        // A button: Increment tag ID
        if (aButtonNow && !lastAButton && targetTagId < 20) {
            targetTagId++;
        }

        // B button: Decrement tag ID
        if (bButtonNow && !lastBButton && targetTagId > -1) {
            targetTagId--;
        }

        // X button: Decrease exposure (darker, less motion blur, better for bright conditions)
        if (xButtonNow) {
            adjustExposure(-1);
        }

        // Y button: Increase exposure (brighter, better for dark conditions)
        if (yButtonNow) {
            adjustExposure(1);
        }

        lastDpadUp = dpadUpNow;
        lastAButton = aButtonNow;
        lastBButton = bButtonNow;

        // Get list of detected AprilTags
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        telemetry.addLine("== WEBCAM APRILTAG TRACKING ==");
        telemetry.addData("Target Tag ID", targetTagId == -1 ? "ANY" : targetTagId);
        telemetry.addData("Tags Detected", currentDetections.size());

        // Process detections
        boolean tagFound = false;

        for (AprilTagDetection detection : currentDetections) {
            // Check if this is the tag we're looking for
            if (targetTagId == -1 || detection.id == targetTagId) {
                tagFound = true;
                lastDetectionTime = System.currentTimeMillis();

                // Get pose information
                if (detection.ftcPose != null) {
                    lastKnownRange = detection.ftcPose.range;
                    lastKnownBearing = detection.ftcPose.bearing;
                    lastKnownYaw = detection.ftcPose.yaw;

                    // Display detection data
                    telemetry.addLine("\n✅ TAG DETECTED");
                    telemetry.addData("Tag ID", detection.id);
                    telemetry.addData("Center Position (x,y)", "(%d, %d)",
                        (int)detection.center.x, (int)detection.center.y);

                    telemetry.addLine("\n== POSITION & ORIENTATION ==");
                    telemetry.addData("Range (Distance)", "%.1f inches", detection.ftcPose.range);
                    telemetry.addData("Bearing (Horizontal)", "%.1f° %s",
                        Math.abs(detection.ftcPose.bearing),
                        detection.ftcPose.bearing > 0 ? "RIGHT" : "LEFT");
                    telemetry.addData("Elevation (Vertical)", "%.1f° %s",
                        Math.abs(detection.ftcPose.elevation),
                        detection.ftcPose.elevation > 0 ? "UP" : "DOWN");

                    telemetry.addLine("\n== TAG ROTATION ==");
                    telemetry.addData("Yaw (Y-axis)", "%.1f°", detection.ftcPose.yaw);
                    telemetry.addData("Pitch (X-axis)", "%.1f°", detection.ftcPose.pitch);
                    telemetry.addData("Roll (Z-axis)", "%.1f°", detection.ftcPose.roll);

                    telemetry.addLine("\n== 3D POSITION (XYZ) ==");
                    telemetry.addData("X (lateral)", "%.1f inches", detection.ftcPose.x);
                    telemetry.addData("Y (forward)", "%.1f inches", detection.ftcPose.y);
                    telemetry.addData("Z (vertical)", "%.1f inches", detection.ftcPose.z);

                    telemetry.addLine("\n== ALIGNMENT GUIDANCE ==");

                    // Horizontal alignment
                    if (Math.abs(detection.ftcPose.bearing) < 3.0) {
                        telemetry.addData("Horizontal", "✅ CENTERED");
                    } else if (detection.ftcPose.bearing > 0) {
                        telemetry.addData("Horizontal", "⬅️ Turn RIGHT %.1f°", detection.ftcPose.bearing);
                    } else {
                        telemetry.addData("Horizontal", "➡️ Turn LEFT %.1f°", Math.abs(detection.ftcPose.bearing));
                    }

                    // Vertical alignment
                    if (Math.abs(detection.ftcPose.elevation) < 3.0) {
                        telemetry.addData("Vertical", "✅ LEVEL");
                    } else if (detection.ftcPose.elevation > 0) {
                        telemetry.addData("Vertical", "⬆️ Aim UP %.1f°", detection.ftcPose.elevation);
                    } else {
                        telemetry.addData("Vertical", "⬇️ Aim DOWN %.1f°", Math.abs(detection.ftcPose.elevation));
                    }

                    // Distance guidance
                    if (detection.ftcPose.range < 12.0) {
                        telemetry.addData("Distance", "⬅️ TOO CLOSE - Back up");
                    } else if (detection.ftcPose.range > 60.0) {
                        telemetry.addData("Distance", "➡️ TOO FAR - Move closer");
                    } else {
                        telemetry.addData("Distance", "✅ IN RANGE");
                    }
                }

                // Only display info for the first matching tag
                break;
            }
        }

        // Check if tag was recently lost
        long timeSinceLastDetection = System.currentTimeMillis() - lastDetectionTime;
        if (!tagFound) {
            if (timeSinceLastDetection < DETECTION_TIMEOUT_MS) {
                telemetry.addLine("\n⚠️ TAG TEMPORARILY LOST");
                telemetry.addData("Last Known Range", "%.1f inches", lastKnownRange);
                telemetry.addData("Last Known Bearing", "%.1f°", lastKnownBearing);
                telemetry.addData("Last Known Yaw", "%.1f°", lastKnownYaw);
            } else {
                telemetry.addLine("\n❌ NO TAG DETECTED");
                telemetry.addData("Searching for Tag", targetTagId == -1 ? "ANY" : String.valueOf(targetTagId));
            }
        }

        telemetry.addLine("\n== CONTROLS ==");
        telemetry.addData("D-pad UP", "Track ANY tag");
        telemetry.addData("A/B", "Adjust target tag ID (+/-)");
        telemetry.addData("X/Y", "Decrease/Increase exposure");

        telemetry.addLine("\n== CAMERA STATUS ==");
        telemetry.addData("FPS", "%.1f", visionPortal.getFps());
        telemetry.addData("Resolution", "640x480 (calibrated)");
        telemetry.addData("Camera State", visionPortal.getCameraState());

        // Show calibration status
        if (currentDetections.isEmpty() && timeSinceLastDetection > DETECTION_TIMEOUT_MS) {
            telemetry.addLine("\n📷 Using built-in calibration");
            telemetry.addLine("C270 @ 640x480 is pre-calibrated");
            telemetry.addLine("Distances should be accurate ±5%");
        }

        telemetry.update();
    }

    // -------------------------------------------------
    // Camera Control Helpers
    // -------------------------------------------------

    private int currentExposure = (int) CameraSettings.exposure;
    private int currentGain = CameraSettings.gain;

    /**
     * Adjust camera exposure for better AprilTag detection in varying lighting
     * Lower exposure = darker image, less motion blur, better for bright conditions
     * Higher exposure = brighter image, better for dark conditions
     *
     * @param direction -1 to decrease, +1 to increase
     */
    private void adjustExposure(int direction) {
        try {
            currentExposure += direction;
            currentExposure = Math.max(1, Math.min(20, currentExposure));  // Clamp 1-20ms

            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl != null) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                exposureControl.setExposure(currentExposure, TimeUnit.MILLISECONDS);
            }

        } catch (Exception e) {
            // Silently fail if control not available
        }
    }

    /**
     * Adjust camera gain for better low-light performance
     *
     * @param gain Value from 0-255 (higher = brighter but more noise)
     */
    private void adjustGain(int gain) {
        try {
            currentGain = Math.max(0, Math.min(255, gain));

            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            if (gainControl != null) {
                gainControl.setGain(currentGain);
            }

        } catch (Exception e) {
            // Silently fail if control not available
        }
    }

    @Override
    public void stop() {
        // Close the vision portal when stopping
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // -------------------------------------------------
    // Helper Methods for Autonomous Use
    // -------------------------------------------------

    /**
     * Gets the last known range (distance) to the tracked AprilTag
     * Useful for autonomous navigation when tag is temporarily obscured
     *
     * @return Range in inches
     */
    public double getLastKnownRange() {
        return lastKnownRange;
    }

    /**
     * Gets the last known bearing to the tracked AprilTag
     * Positive = right, Negative = left
     *
     * @return Bearing in degrees
     */
    public double getLastKnownBearing() {
        return lastKnownBearing;
    }

    /**
     * Gets the last known yaw of the tracked AprilTag
     *
     * @return Yaw in degrees
     */
    public double getLastKnownYaw() {
        return lastKnownYaw;
    }

    /**
     * Checks if the target tag is currently visible
     *
     * @return true if tag was detected within the timeout period
     */
    public boolean isTagVisible() {
        return (System.currentTimeMillis() - lastDetectionTime) < DETECTION_TIMEOUT_MS;
    }

    /**
     * Checks if the robot is aligned with the target tag (horizontally)
     *
     * @param bearingTolerance Acceptable bearing error in degrees
     * @return true if bearing is within tolerance
     */
    public boolean isAlignedWithTag(double bearingTolerance) {
        return isTagVisible() && Math.abs(lastKnownBearing) <= bearingTolerance;
    }

    /**
     * Checks if the robot is at the target distance from the tag
     *
     * @param targetRange Desired distance in inches
     * @param rangeTolerance Acceptable distance error in inches
     * @return true if range is within tolerance
     */
    public boolean isAtTargetRange(double targetRange, double rangeTolerance) {
        return isTagVisible() &&
               Math.abs(lastKnownRange - targetRange) <= rangeTolerance;
    }

    /**
     * Gets the current AprilTag detections list
     * Useful for processing multiple tags in autonomous
     *
     * @return List of currently detected AprilTags
     */
    public List<AprilTagDetection> getCurrentDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Finds a specific tag by ID from current detections
     *
     * @param tagId The ID of the tag to find
     * @return AprilTagDetection if found, null otherwise
     */
    public AprilTagDetection getTagById(int tagId) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == tagId) {
                return detection;
            }
        }
        return null;
    }
}
