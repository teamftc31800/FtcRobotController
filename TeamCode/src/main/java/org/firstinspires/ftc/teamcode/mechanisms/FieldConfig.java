package org.firstinspires.ftc.teamcode.mechanisms;

/**
 * Season-specific AprilTag field positions and camera mounting offsets.
 *
 * This is the ONLY file that changes each season. To support a new season:
 *   1. Add a new static factory method (e.g., newSeason2026())
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
     * Into the Deep 2024-2025 season.
     *
     * Tag positions are in Pedro Pathing field coordinates (inches).
     * headingDeg = direction the tag face points outward from the wall.
     *
     * TODO: Replace placeholder positions with measured values from the
     *       game manual, converted to your Pedro Pathing coordinate frame.
     *       The FTC field is 144" x 144". Pedro Pathing origin is typically
     *       at one corner with +X and +Y going into the field.
     */
    public static FieldConfig intoTheDeep2025() {
        return new FieldConfig(
                6.0,   // camera ~6" forward of robot center (measure on your robot)
                0.0,   // camera centered left-right (measure on your robot)
                new TagFieldPose[] {
                        // Blue alliance tags
                        new TagFieldPose(20,   0.0,  72.0,   0.0),  // TODO: measure actual position
                        new TagFieldPose(21,   0.0,  48.0,   0.0),  // TODO: measure actual position
                        // Shared / center tags
                        new TagFieldPose(22,  72.0, 144.0,  270.0), // TODO: measure actual position
                        new TagFieldPose(23,  72.0,   0.0,   90.0), // TODO: measure actual position
                        // Red alliance tags
                        new TagFieldPose(24, 144.0,  72.0,  180.0), // TODO: measure actual position
                }
        );
    }

    // Next season example:
    // public static FieldConfig newSeason2026() { ... }
}
