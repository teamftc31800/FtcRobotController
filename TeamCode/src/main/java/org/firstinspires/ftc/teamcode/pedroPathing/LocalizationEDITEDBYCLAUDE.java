package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagPoseMathCLAUDE;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.FieldConfig;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

/**
 * Fuses Pedro Pathing's {@link Follower} pose (driven every loop by the
 * goBILDA Pinpoint via {@link Constants#localizerConstants}) with AprilTag
 * detections to correct the drift that accumulates in odometry alone.
 *
 * Pinpoint is fast and continuous but drifts over a match. AprilTags are
 * intermittent but give an absolute field position. This class periodically
 * nudges the Follower's pose estimate toward the AprilTag-derived pose so
 * path following stays accurate late into autonomous or TeleOp.
 *
 * Season-agnostic: tag field positions and camera offsets come from
 * {@link FieldConfig}, the only file that changes each year.
 *
 * Usage:
 *   follower = Constants.createFollower(hardwareMap);
 *   follower.setStartingPose(startPose);
 *   aprilTag = new AprilTagWebcam();
 *   aprilTag.init(hardwareMap, telemetry);
 *   localization = new Localization(follower, aprilTag, FieldConfig.biobuzz2026_2027(), 0.3);
 *
 *   // in loop():
 *   follower.update();
 *   aprilTag.update();
 *   localization.update();
 */
public class LocalizationEDITEDBYCLAUDE {

    private final Follower follower;
    private final AprilTagWebcam aprilTagWebcam;
    private final FieldConfig fieldConfig;

    // Correction weight: 0.0 = trust only Pinpoint, 1.0 = snap fully to camera
    private double correctionAlpha;

    // Reject camera pose estimates that disagree with Pinpoint by more than this (inches)
    private static final double MAX_CORRECTION_INCHES = 24.0;

    // Diagnostics
    private boolean lastUpdateHadTagCorrection;
    private int tagCorrectionCount;
    private double lastCorrectionDeltaX;
    private double lastCorrectionDeltaY;

    /**
     * @param follower        Follower already created via Constants.createFollower(hardwareMap)
     * @param aprilTagWebcam  initialized AprilTagWebcam instance
     * @param fieldConfig     season-specific tag positions and camera offsets
     * @param correctionAlpha blend weight for camera corrections (0.0-1.0, recommend 0.3)
     */
    public LocalizationEDITEDBYCLAUDE( Follower follower, AprilTagWebcam aprilTagWebcam, FieldConfig fieldConfig, double correctionAlpha) {
        this.follower = follower;
        this.aprilTagWebcam = aprilTagWebcam;
        this.fieldConfig = fieldConfig;
        this.correctionAlpha = correctionAlpha;
    }

    /**
     * Call every loop iteration, after follower.update() and aprilTagWebcam.update().
     */
    public void update() {
        lastUpdateHadTagCorrection = false;

        List<AprilTagDetection> detections = aprilTagWebcam.getDetectedTags();
        if (detections == null || detections.isEmpty()) return;

        Pose currentPose = follower.getPose();
        double curX = currentPose.getX();
        double curY = currentPose.getY();
        double curHeadingDeg = Math.toDegrees(currentPose.getHeading());

        double sumX = 0, sumY = 0, sumHeading = 0;
        int count = 0;

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose == null) continue;

            FieldConfig.TagFieldPose tagPose = fieldConfig.getTagById(detection.id);
            if (tagPose == null) continue;

            double[] robotPose = AprilTagPoseMathCLAUDE.computeRobotPoseFromTag(
                    tagPose.x, tagPose.y, tagPose.headingDeg,
                    fieldConfig.cameraForwardOffset, fieldConfig.cameraRightOffset,
                    detection.ftcPose.range,
                    detection.ftcPose.bearing,
                    detection.ftcPose.yaw
            );

            // Sanity check: reject wild outliers
            double dx = robotPose[0] - curX;
            double dy = robotPose[1] - curY;
            if (Math.sqrt(dx * dx + dy * dy) > MAX_CORRECTION_INCHES) continue;

            sumX += robotPose[0];
            sumY += robotPose[1];
            sumHeading += robotPose[2];
            count++;
        }

        if (count == 0) return;

        double tagX = sumX / count;
        double tagY = sumY / count;
        double tagHeadingDeg = sumHeading / count;

        // Weighted blend toward camera estimate
        lastCorrectionDeltaX = tagX - curX;
        lastCorrectionDeltaY = tagY - curY;

        double newX = curX + correctionAlpha * lastCorrectionDeltaX;
        double newY = curY + correctionAlpha * lastCorrectionDeltaY;
        double newHeadingDeg = curHeadingDeg
                + correctionAlpha * AprilTagPoseMathCLAUDE.normalizeAngle(tagHeadingDeg - curHeadingDeg);

        follower.setPose(new Pose(newX, newY, Math.toRadians(newHeadingDeg)));

        lastUpdateHadTagCorrection = true;
        tagCorrectionCount++;
    }

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
}
