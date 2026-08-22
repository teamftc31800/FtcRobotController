package org.firstinspires.ftc.teamcode.mechanisms;

/**
 * Shared math for turning an AprilTag detection (range/bearing/yaw relative to
 * the camera) plus that tag's known field position into an absolute robot
 * field pose. Used by both {@link RobotLocalizerEDITEDBYCLAUDE} (standalone Pinpoint fusion)
 * and {@code org.firstinspires.ftc.teamcode.pedroPathing.Localization} (Pedro
 * Follower fusion) so the geometry lives in exactly one place.
 */
public final class AprilTagPoseMathCLAUDE {

    private AprilTagPoseMathCLAUDE() {}

    /**
     * Given a tag's known field position and the camera's detection of that tag,
     * compute where the robot must be on the field.
     *
     * @param tagFieldX         tag field X (inches)
     * @param tagFieldY         tag field Y (inches)
     * @param tagFieldHeadingDeg outward-facing normal of the tag (degrees)
     * @param cameraForwardOffset camera offset forward of robot center (inches)
     * @param cameraRightOffset   camera offset right of robot center (inches)
     * @param range   detection range (inches)
     * @param bearing detection bearing (degrees, + = right of camera center)
     * @param yaw     detection yaw (degrees, + = tag rotated CW from camera's view)
     * @return double[3] = { robotX, robotY, robotHeadingDeg }
     */
    public static double[] computeRobotPoseFromTag(
            double tagFieldX, double tagFieldY, double tagFieldHeadingDeg,
            double cameraForwardOffset, double cameraRightOffset,
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
        double camOffX = cameraForwardOffset * cosH - cameraRightOffset * sinH;
        double camOffY = cameraForwardOffset * sinH + cameraRightOffset * cosH;

        // Step 5: Robot position = tag position - camera-to-tag vector - camera offset
        double robotX = tagFieldX - fieldDx - camOffX;
        double robotY = tagFieldY - fieldDy - camOffY;

        return new double[]{ robotX, robotY, robotHeadingDeg };
    }

    /** Normalize angle to [-180, +180]. */
    public static double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees > 180.0) degrees -= 360.0;
        if (degrees <= -180.0) degrees += 360.0;
        return degrees;
    }
}
