package org.firstinspires.ftc.teamcode.mechanisms;

/**
 * Season-specific AprilTag field positions and camera mounting offsets.
 *
 * This is the ONLY file that changes each season. To support a new season:
 *   1. Add a new static factory method (e.g., biobuzz2026_2027())
 *   2. Fill in tag IDs, field positions, and camera offsets
 *   3. Update the FieldConfig call in your TeleOp init()
 *
 * Coordinate system must match Pedro Pathing (origin at field corner, inches).
 */
public class FieldConfig {

    /** One AprilTag's known position on the field. */
    public static class TagFieldPose {
        public final int id;
        public final double x;          // inches, field coordinates
        public final double y;          // inches, field coordinates
        public final double headingDeg; // outward-facing normal of the tag (degrees)

        public TagFieldPose(int id, double x, double y, double headingDeg) {
            this.id = id;
            this.x = x;
            this.y = y;
            this.headingDeg = headingDeg;
        }
    }

    /** Camera offset from robot center of rotation (inches). */
    public final double cameraForwardOffset;  // positive = camera is in front of center
    public final double cameraRightOffset;    // positive = camera is to the right of center

    /** All tags on the field this season. */
    public final TagFieldPose[] tags;


    // Creates a constructer with 3 instance fields parameters
    private FieldConfig(double cameraForwardOffset, double cameraRightOffset, TagFieldPose[] tags) {
        this.cameraForwardOffset = cameraForwardOffset;
        this.cameraRightOffset = cameraRightOffset;
        this.tags = tags;
    }

    /** Look up a tag by ID. Returns null if not in this season's config. */
    public TagFieldPose getTagById(int id) {
        for (TagFieldPose tag : tags) {
            if (tag.id == id) return tag;
        }
        return null;
    }

    // =======================================================================
    // SEASON FACTORY METHODS — add one per season
    // =======================================================================

    /**
     * Biobuzz 2026-2027 season.
     *
     * IMPORTANT — Biobuzz has two kinds of AprilTags, and only one kind
     * belongs in this list:
     *
     *   - FLOWER tags: 4 tags fixed to the field perimeter. These behave
     *     like a normal season's tags and are safe for absolute-position
     *     correction — put THESE in the list below.
     *
     *   - HIVE cluster tags (reported ID range ~30-45, 4 tags per cluster
     *     on the CELL bottoms, 3.25" size, 36h11 family): mounted on the
     *     HIVE structure, which physically tips between two stable CELL
     *     orientations during a match. Their field position is NOT fixed,
     *     so DO NOT add them here — doing so would silently feed a wrong
     *     "known" position into computeRobotPoseFromTag() every time the
     *     HIVE tips, corrupting the fused pose. If you ever want to use
     *     HIVE tags (e.g. for aiming at a CELL), that needs separate,
     *     dedicated logic — not FieldConfig/RobotLocalizer correction.
     *
     * TODO: The entries below are placeholders — the exact FLOWER tag IDs
     *       and their measured (x, y, headingDeg) have NOT been verified
     *       against the official Competition Manual (Section 9, "Arena",
     *       Figure 9-17) yet. Replace them with the real values from that
     *       figure before trusting this for localization.
     *
     * Tag positions are in Pedro Pathing field coordinates (inches).
     * headingDeg = direction the tag face points outward from the wall.
     * The FTC field is 144" x 144". Pedro Pathing origin is typically
     * at one corner with +X and +Y going into the field.
     */
    public static FieldConfig biobuzz2026_2027() {
        return new FieldConfig(
                6.0,   // camera ~6" forward of robot center (measure on your robot)
                0.0,   // camera centered left-right (measure on your robot)
                new TagFieldPose[] {
                        // FLOWER tags (fixed, field perimeter) — placeholders, see TODO above
                        new TagFieldPose(20,   0.0,  72.0,   0.0),  // TODO: verify ID + measure actual position
                        new TagFieldPose(21,  72.0, 144.0,  270.0), // TODO: verify ID + measure actual position
                        new TagFieldPose(22,  72.0,   0.0,   90.0), // TODO: verify ID + measure actual position
                        new TagFieldPose(23, 144.0,  72.0,  180.0), // TODO: verify ID + measure actual position

                        // Do NOT add HIVE cluster tags (~30-45) here — see the
                        // class-level comment above for why they'd break localization.
                }
        );
    }

    // Next season example:
    // public static FieldConfig nextSeason() { ... }
}
