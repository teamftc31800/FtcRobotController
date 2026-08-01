package org.firstinspires.ftc.teamcode.mechanisms;

/**
 * Distance-based auto-adjustment for arm angle and flywheel RPM.
 *
 * Uses an exact lookup table — if the robot is within 2.5" of a known
 * data point, it snaps to that point's RPM and angle exactly.
 *
 * Field-tested data:
 *
 * Distance (in) | RPM  | Angle (°)
 * --------------|------|----------
 *     45        | 2350 |   150
 *     50        | 2400 |   150
 *     55        | 2450 |   120
 *     60        | 2500 |   120
 *     65        | 2550 |   120
 *     70        | 2560 |   120
 *     75        | 2650 |   120
 *     80        | 2750 |   120
 *     85        | 2750 |   120
 *     90        | 2825 |   120
 *     95        | 2900 |   120
 *    100        | 2965 |   120
 */
public class RPM_per_dist {

    private static final double SNAP_TOLERANCE = 2.5;  // inches

    // =====================================================
    // LOOKUP TABLE  (distance, rpm, angle)
    // =====================================================
    private static final double[][] TABLE = {
        //  dist    rpm     angle
        {   45.0,  2350.0,  150.0 },
        {   50.0,  2400.0,  150.0 },
        {   55.0,  2450.0,  120.0 },
        {   60.0,  2500.0,  120.0 },
        {   65.0,  2550.0,  120.0 },
        {   70.0,  2560.0,  120.0 },
        {   75.0,  2650.0,  120.0 },
        {   80.0,  2750.0,  120.0 },
        {   85.0,  2750.0,  120.0 },
        {   90.0,  2825.0,  120.0 },
        {   95.0,  2900.0,  120.0 },
        {  100.0,  2965.0,  120.0 },
    };

    // =====================================================
    // DISTANCE → ARM ANGLE
    // =====================================================
    /**
     * Returns the arm angle for the closest matching data point (within 2.5").
     * If outside the table range, extrapolates linearly from the nearest two endpoints.
     */
    public double getArmAngleForDistance(double distanceInches) {
        int idx = findClosestIndex(distanceInches);
        if (idx >= 0) return TABLE[idx][2];

        // Out of range: extrapolate from the nearest two endpoints
        if (distanceInches < TABLE[0][0]) {
            // Below table minimum — extrapolate from first two rows
            double slope = (TABLE[1][2] - TABLE[0][2]) / (TABLE[1][0] - TABLE[0][0]);
            return TABLE[0][2] + slope * (distanceInches - TABLE[0][0]);
        } else {
            // Above table maximum — extrapolate from last two rows
            int last = TABLE.length - 1;
            double slope = (TABLE[last][2] - TABLE[last - 1][2]) / (TABLE[last][0] - TABLE[last - 1][0]);
            return TABLE[last][2] + slope * (distanceInches - TABLE[last][0]);
        }
    }

    // =====================================================
    // DISTANCE → FLYWHEEL RPM
    // =====================================================
    /**
     * Returns the flywheel RPM for the closest matching data point (within 2.5").
     * If outside the table range, extrapolates linearly from the nearest two endpoints.
     */
    public double getFlywheelRPMForDistance(double distanceInches) {
        int idx = findClosestIndex(distanceInches);
        if (idx >= 0) return TABLE[idx][1];

        // Out of range: extrapolate from the nearest two endpoints
        if (distanceInches < TABLE[0][0]) {
            // Below table minimum — extrapolate from first two rows
            double slope = (TABLE[1][1] - TABLE[0][1]) / (TABLE[1][0] - TABLE[0][0]);
            return TABLE[0][1] + slope * (distanceInches - TABLE[0][0]);
        } else {
            // Above table maximum — extrapolate from last two rows
            int last = TABLE.length - 1;
            double slope = (TABLE[last][1] - TABLE[last - 1][1]) / (TABLE[last][0] - TABLE[last - 1][0]);
            return TABLE[last][1] + slope * (distanceInches - TABLE[last][0]);
        }
    }

    // =====================================================
    // HELPER: Shooting mode label for telemetry
    // =====================================================
    public String getShootingMode(double distanceInches) {
        int idx = findClosestIndex(distanceInches);
        if (idx < 0) {
            if (distanceInches < TABLE[0][0]) return String.format("EXTRAP (close) %.0f\"", distanceInches);
            return String.format("EXTRAP (far) %.0f\"", distanceInches);
        }
        double snappedDist = TABLE[idx][0];
        double angle       = TABLE[idx][2];
        return String.format("%.0f\" → %.0f°", snappedDist, angle);
    }

    // =====================================================
    // HELPER: Find index of closest data point within 2.5"
    // Returns -1 if nothing is within tolerance
    // =====================================================
    private int findClosestIndex(double distanceInches) {
        int    bestIdx  = -1;
        double bestDiff = SNAP_TOLERANCE + 1.0;  // start outside tolerance

        for (int i = 0; i < TABLE.length; i++) {
            double diff = Math.abs(distanceInches - TABLE[i][0]);
            if (diff <= SNAP_TOLERANCE && diff < bestDiff) {
                bestDiff = diff;
                bestIdx  = i;
            }
        }
        return bestIdx;
    }
}
