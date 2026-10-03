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
public class RobotLocalizerEDITEDBYCLAUDE {

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

    // Reject camera estimates that differ from odometry by more than this (inches).
    // Not final: widen it with setMaxCorrectionInches() when deliberately testing
    // with a tag coordinate that's far from where odometry thinks the robot is.
    private double maxCorrectionInches = 24.0;

    // Diagnostics
    private boolean lastUpdateHadTagCorrection;
    private int tagCorrectionCount;
    private double lastCorrectionDeltaX;
    private double lastCorrectionDeltaY;

    // Raw (unblended) pose computed from the tag(s) this update, before the
    // outlier check and before easing toward it — lets you compare the math's
    // raw output against your own hand-calculated expected value.
    private boolean lastUpdateSawUsableTag;
    private boolean lastUpdateRejectedAsOutlier;
    private double lastTagOnlyX;
    private double lastTagOnlyY;
    private double lastTagOnlyHeadingDeg;

    /**
     * Constructor — sets up the localizer with a starting position and takes
     * a "snapshot" of the current odometry reading so the very first call to
     * update() doesn't see a huge fake jump in position.
     *
     * @param odometry        initialized PinpointOdometry instance
     * @param fieldConfig     season-specific tag positions and camera offsets
     * @param startX          starting X position on field (inches)
     * @param startY          starting Y position on field (inches)
     * @param startHeadingDeg starting heading (degrees)
     * @param correctionAlpha blend weight for camera corrections (0.0-1.0, recommend 0.3)
     */
    public RobotLocalizerEDITEDBYCLAUDE(
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

        // Snapshot current odometry so the first delta computed in update() is zero
        odometry.update();
        this.lastOdoX = odometry.getX();
        this.lastOdoY = odometry.getY();
        this.lastOdoHeading = odometry.getHeadingDeg();
    }

    /**
     * The main loop method — call this once every loop iteration.
     * It does two jobs, in order:
     *   1. Always trust Pinpoint for how far the robot moved since last loop
     *      (fast and smooth, but slowly drifts over time).
     *   2. If any AprilTags are visible this loop, nudge our position toward
     *      what the camera says is true (slow/intermittent, but doesn't drift).
     *
     * @param detections current AprilTag detections (may be null or empty)
     */
    public void update(List<AprilTagDetection> detections) {

        // --- Step 1: Apply odometry delta ---
        // Read the Pinpoint's current position, figure out how much it moved
        // since the last loop (the "delta"), and add that delta onto our
        // running fused position. We use deltas instead of just copying
        // Pinpoint's value directly so that a camera correction applied last
        // loop (Step 2) doesn't get overwritten/erased this loop.
        odometry.update();
        double odoX = odometry.getX();
        double odoY = odometry.getY();
        double odoHeading = odometry.getHeadingDeg();

        x += odoX - lastOdoX;
        y += odoY - lastOdoY;
        headingDeg = normalizeAngle(headingDeg + (odoHeading - lastOdoHeading));

        // Remember this loop's odometry reading so next loop can compute its own delta
        lastOdoX = odoX;
        lastOdoY = odoY;
        lastOdoHeading = odoHeading;







        // --- Step 2: AprilTag correction ---
        lastUpdateHadTagCorrection = false;
        lastUpdateSawUsableTag = false;
        lastUpdateRejectedAsOutlier = false;

        // No tags seen this loop -> nothing to correct, keep the odometry-only position
        if (detections == null || detections.isEmpty()) return;

        // rawSum* includes every detection with a known tag ID, regardless of
        // distance from the current fused pose — this is what "the math says",
        // for debugging. sum*/count is the same but with outliers removed,
        // and is what actually gets blended into the fused pose below.
        double rawSumX = 0, rawSumY = 0, rawSumHeading = 0;
        int rawCount = 0;
        double sumX = 0, sumY = 0, sumHeading = 0;
        int count = 0;

        // A single loop can see multiple tags at once — turn each one into a
        // "here's where I think the robot is" estimate, then average them.
        for (AprilTagDetection detection : detections) {
            // ftcPose is null if the camera saw the tag but couldn't solve its 3D pose
            if (detection.ftcPose == null) continue;

            // Skip tags we don't have a known field position for (e.g. wrong season/id)
            FieldConfig.TagFieldPose tagPose = fieldConfig.getTagById(detection.id);
            if (tagPose == null) continue;

            // Combine "where the tag lives on the field" with "how far/what angle
            // the camera saw it at" to back-calculate where the robot must be
            double[] robotPose = AprilTagPoseMathCLAUDE.computeRobotPoseFromTag(
                    tagPose.x, tagPose.y, tagPose.headingDeg,
                    fieldConfig.cameraForwardOffset, fieldConfig.cameraRightOffset,
                    detection.ftcPose.range,
                    detection.ftcPose.bearing,
                    detection.ftcPose.yaw
            );

            rawSumX += robotPose[0];
            rawSumY += robotPose[1];
            rawSumHeading += robotPose[2];
            rawCount++;

            // Sanity check: reject wild outliers (e.g. misidentified tag, bad
            // reflection) that disagree with odometry by more than
            // maxCorrectionInches, so one bad frame can't teleport the
            // robot's tracked position
            double dx = robotPose[0] - x;
            double dy = robotPose[1] - y;
            if (Math.sqrt(dx * dx + dy * dy) > maxCorrectionInches) continue;

            sumX += robotPose[0];
            sumY += robotPose[1];
            sumHeading += robotPose[2];
            count++;
        }

        if (rawCount > 0) {
            lastUpdateSawUsableTag = true;
            lastTagOnlyX = rawSumX / rawCount;
            lastTagOnlyY = rawSumY / rawCount;
            lastTagOnlyHeadingDeg = rawSumHeading / rawCount;
        }

        // Every detection this loop was rejected/unusable -> nothing to blend in
        if (count == 0) {
            // If we saw a usable tag but it still got thrown out, it was the
            // outlier filter (maxCorrectionInches), not a missing/unknown tag ID
            lastUpdateRejectedAsOutlier = rawCount > 0;
            return;
        }

        // Average all the surviving per-tag estimates into one camera pose
        double tagX = sumX / count;
        double tagY = sumY / count;
        double tagHeading = sumHeading / count;

        // Weighted blend toward camera estimate. We don't snap straight to the
        // camera's number (that would cause visible jumps/jitter) — instead we
        // move a fraction (correctionAlpha) of the way there each loop, so the
        // position eases toward "truth" smoothly over a few loops.
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
    // Getters
    // ---------------------------------------------------------------

    /** Fused field X position, in inches — the best current guess after odometry + camera blending. */
    public double getX() { return x; }
    /** Fused field Y position, in inches — the best current guess after odometry + camera blending. */
    public double getY() { return y; }
    /** Fused heading, in degrees — the best current guess after odometry + camera blending. */
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

    /** True if at least one detected tag had a known FieldConfig entry this update (even if later rejected as an outlier). */
    public boolean hadUsableTagThisUpdate() { return lastUpdateSawUsableTag; }

    /** True if a usable tag was seen but every candidate pose was farther than maxCorrectionInches from the current fused pose. */
    public boolean wasLastTagRejectedAsOutlier() { return lastUpdateRejectedAsOutlier; }

    /** Raw field X computed from the tag(s) this update, before outlier rejection and before easing — only valid if hadUsableTagThisUpdate(). */
    public double getLastTagOnlyX() { return lastTagOnlyX; }

    /** Raw field Y computed from the tag(s) this update, before outlier rejection and before easing — only valid if hadUsableTagThisUpdate(). */
    public double getLastTagOnlyY() { return lastTagOnlyY; }

    /** Raw heading computed from the tag(s) this update, before outlier rejection and before easing — only valid if hadUsableTagThisUpdate(). */
    public double getLastTagOnlyHeadingDeg() { return lastTagOnlyHeadingDeg; }

    // ---------------------------------------------------------------
    // Tuning
    // ---------------------------------------------------------------

    /**
     * Adjusts how strongly camera corrections pull the fused position each loop.
     * 0.0 = ignore the camera entirely (pure odometry). 1.0 = snap fully to the
     * camera's estimate every time a tag is seen (can look jumpy). Value is
     * clamped into the valid 0.0-1.0 range so a typo can't break the blend math.
     */
    public void setCorrectionAlpha(double alpha) {
        this.correctionAlpha = Math.max(0.0, Math.min(1.0, alpha));
    }

    /**
     * Widens or tightens the outlier-rejection distance (inches). Raise this
     * (or pass a huge value) when deliberately testing with a tag coordinate
     * far from the robot's current position, so the correction isn't silently
     * thrown out. Lower/leave at the default for real matches, so a
     * misidentified tag can't teleport the tracked position.
     */
    public void setMaxCorrectionInches(double inches) {
        this.maxCorrectionInches = Math.max(0.0, inches);
    }

    public double getMaxCorrectionInches() { return maxCorrectionInches; }

    /**
     * Hard reset — use when you know the exact position (e.g., after auto).
     * Unlike the normal odometry-delta blending in update(), this directly
     * overwrites the fused pose and re-snapshots odometry, so no correction
     * math or smoothing is applied.
     */
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

    /**
     * Normalize angle to [-180, +180]. Needed because angles wrap around
     * (e.g. 350 degrees and -10 degrees are the same heading) — without this,
     * subtracting two headings near the wraparound point could produce a huge
     * fake jump (like 350) instead of the small true difference (like -20).
     */
    private static double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees > 180.0) degrees -= 360.0;
        if (degrees <= -180.0) degrees += 360.0;
        return degrees;
    }
}
