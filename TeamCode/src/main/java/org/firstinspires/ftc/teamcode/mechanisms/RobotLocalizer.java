package org.firstinspires.ftc.teamcode.mechanisms;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Fuses Pinpoint odometry (fast, drifts) with AprilTag camera (intermittent, absolute)
 * to produce the robot's field position in inches.
 *
 * Season-agnostic: tag positions come from FieldConfig, which is the only
 * file that changes each year.
 *
 * Usage:
 *   localizer = new RobotLocalizer(odometry, fieldConfig, startX, startY, startHeading, 0.3);
 *   // in loop():
 *   localizer.update(aprilTag.getDetections());
 *   double x = localizer.getX();  // inches
 */
public class RobotLocalizer {

    private final PinpointOdometry odometry;
    private final FieldConfig fieldConfig;

    // Current fused pose (field coordinates, inches/degrees)
    private double x;
    private double y;
    private double headingDeg;

    // Previous odometry readings (for computing deltas)
    private double lastOdoX;
    private double lastOdoY;
    private double lastOdoHeading;

    // Correction weight: 0.0 = trust only odometry, 1.0 = trust only camera
    private double correctionAlpha;

    // Reject camera estimates that differ from odometry by more than this (inches)
    private static final double MAX_CORRECTION_INCHES = 24.0;

    // Diagnostics
    private boolean lastUpdateHadTagCorrection;
    private int tagCorrectionCount;
    private double lastCorrectionDeltaX;
    private double lastCorrectionDeltaY;

    /**
     * @param odometry        initialized PinpointOdometry instance
     * @param fieldConfig     season-specific tag positions and camera offsets
     * @param startX          starting X position on field (inches)
     * @param startY          starting Y position on field (inches)
     * @param startHeadingDeg starting heading (degrees)
     * @param correctionAlpha blend weight for camera corrections (0.0-1.0, recommend 0.3)
     */
    public RobotLocalizer(
            PinpointOdometry odometry,
            FieldConfig fieldConfig,
            double startX, double startY, double startHeadingDeg,
            double correctionAlpha
    ) {
        this.odometry = odometry;
        this.fieldConfig = fieldConfig;
        this.x = startX;
        this.y = startY;
        this.headingDeg = startHeadingDeg;
        this.correctionAlpha = correctionAlpha;

        // Snapshot current odometry so first delta is zero
        odometry.update();
        this.lastOdoX = odometry.getX();
        this.lastOdoY = odometry.getY();
        this.lastOdoHeading = odometry.getHeadingDeg();
    }

    /**
     * Call every loop iteration.
     * @param detections current AprilTag detections (may be null or empty)
     */
    public void update(List<AprilTagDetection> detections) {

        // --- Step 1: Apply odometry delta ---
        odometry.update();
        double odoX = odometry.getX();
        double odoY = odometry.getY();
        double odoHeading = odometry.getHeadingDeg();

        x += odoX - lastOdoX;
        y += odoY - lastOdoY;
        headingDeg = normalizeAngle(headingDeg + (odoHeading - lastOdoHeading));

        lastOdoX = odoX;
        lastOdoY = odoY;
        lastOdoHeading = odoHeading;

        // --- Step 2: AprilTag correction ---
        lastUpdateHadTagCorrection = false;

        if (detections == null || detections.isEmpty()) return;

        double sumX = 0, sumY = 0, sumHeading = 0;
        int count = 0;

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose == null) continue;

            FieldConfig.TagFieldPose tagPose = fieldConfig.getTagById(detection.id);
            if (tagPose == null) continue;

            double[] robotPose = computeRobotPoseFromTag(
                    tagPose.x, tagPose.y, tagPose.headingDeg,
                    detection.ftcPose.range,
                    detection.ftcPose.bearing,
                    detection.ftcPose.yaw
            );

            // Sanity check: reject wild outliers
            double dx = robotPose[0] - x;
            double dy = robotPose[1] - y;
            if (Math.sqrt(dx * dx + dy * dy) > MAX_CORRECTION_INCHES) continue;

            sumX += robotPose[0];
            sumY += robotPose[1];
            sumHeading += robotPose[2];
            count++;
        }

        if (count == 0) return;

        double tagX = sumX / count;
        double tagY = sumY / count;
        double tagHeading = sumHeading / count;

        // Weighted blend toward camera estimate
        lastCorrectionDeltaX = tagX - x;
        lastCorrectionDeltaY = tagY - y;

        x += correctionAlpha * lastCorrectionDeltaX;
        y += correctionAlpha * lastCorrectionDeltaY;
        headingDeg = normalizeAngle(
                headingDeg + correctionAlpha * normalizeAngle(tagHeading - headingDeg)
        );

        lastUpdateHadTagCorrection = true;
        tagCorrectionCount++;
    }

    // ---------------------------------------------------------------
    // AprilTag → robot field position math
    // ---------------------------------------------------------------

    /**
     * Given a tag's known field position and the camera's detection of that tag,
     * compute where the robot must be on the field.
     *
     * @return double[3] = { robotX, robotY, robotHeadingDeg }
     */
    private double[] computeRobotPoseFromTag(
            double tagFieldX, double tagFieldY, double tagFieldHeadingDeg,
            double range, double bearing, double yaw
    ) {
        // Step 1: Robot heading — tag faces outward at tagFieldHeadingDeg.
        // If camera looks straight at the tag face (yaw=0), camera points
        // opposite to tag normal → cameraHeading = tagHeading + 180.
        // yaw rotates that: positive yaw = tag rotated CW from camera's view.
        double robotHeadingDeg = normalizeAngle(tagFieldHeadingDeg + 180.0 - yaw);
        double robotHeadingRad = Math.toRadians(robotHeadingDeg);

        // Step 2: Camera-to-tag vector in camera frame
        // bearing: positive = tag is to the right of camera center
        double bearingRad = Math.toRadians(bearing);
        double dxCam = range * Math.sin(bearingRad);   // + = right
        double dyCam = range * Math.cos(bearingRad);   // + = forward

        // Step 3: Rotate camera-frame vector into field frame
        double cosH = Math.cos(robotHeadingRad);
        double sinH = Math.sin(robotHeadingRad);
        double fieldDx = dxCam * cosH - dyCam * sinH;
        double fieldDy = dxCam * sinH + dyCam * cosH;

        // Step 4: Camera offset from robot center, in field frame
        double camFwd = fieldConfig.cameraForwardOffset;
        double camRight = fieldConfig.cameraRightOffset;
        double camOffX = camFwd * cosH - camRight * sinH;
        double camOffY = camFwd * sinH + camRight * cosH;

        // Step 5: Robot position = tag position - camera-to-tag vector - camera offset
        double robotX = tagFieldX - fieldDx - camOffX;
        double robotY = tagFieldY - fieldDy - camOffY;

        return new double[]{ robotX, robotY, robotHeadingDeg };
    }

    // ---------------------------------------------------------------
    // Getters
    // ---------------------------------------------------------------

    public double getX() { return x; }
    public double getY() { return y; }
    public double getHeadingDeg() { return headingDeg; }

    // ---------------------------------------------------------------
    // Diagnostics (for telemetry)
    // ---------------------------------------------------------------

    /** True if the most recent update() applied a camera correction. */
    public boolean hadTagCorrection() { return lastUpdateHadTagCorrection; }

    /** Total number of loops where a camera correction was applied. */
    public int getTagCorrectionCount() { return tagCorrectionCount; }

    /** X component of the most recent correction (inches). */
    public double getLastCorrectionDeltaX() { return lastCorrectionDeltaX; }

    /** Y component of the most recent correction (inches). */
    public double getLastCorrectionDeltaY() { return lastCorrectionDeltaY; }

    // ---------------------------------------------------------------
    // Tuning
    // ---------------------------------------------------------------

    public void setCorrectionAlpha(double alpha) {
        this.correctionAlpha = Math.max(0.0, Math.min(1.0, alpha));
    }

    /** Hard reset — use when you know the exact position (e.g., after auto). */
    public void resetPose(double newX, double newY, double newHeadingDeg) {
        this.x = newX;
        this.y = newY;
        this.headingDeg = newHeadingDeg;
        odometry.update();
        this.lastOdoX = odometry.getX();
        this.lastOdoY = odometry.getY();
        this.lastOdoHeading = odometry.getHeadingDeg();
    }

    // ---------------------------------------------------------------
    // Utility
    // ---------------------------------------------------------------

    /** Normalize angle to [-180, +180]. */
    private static double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees > 180.0) degrees -= 360.0;
        if (degrees <= -180.0) degrees += 360.0;
        return degrees;
    }
}
