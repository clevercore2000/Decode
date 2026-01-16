package org.firstinspires.ftc.teamcode.Utils;

/**
 * Math utilities for swerve drive angle calculations.
 *
 * CRITICAL for 1:2 gear ratio swerve:
 * - Encoder reads 0-720° (servo) → wheel angle = servo/2 = 0-360°
 * - Error calculation uses [-180°, 180°] range for shortest path
 * - When wheel goes from 359° to 1°, error should be +2°, NOT -358°
 */
public class MathUtils {

    private static final double TWO_PI = 2.0 * Math.PI;

    /**
     * Normalizes an angle to the range [-180, 180] degrees.
     * Uses modulo arithmetic for O(1) performance (no loops).
     *
     * Examples:
     *   359° → -1° (shortest path is -1°, not +359°)
     *   -270° → 90°
     *   540° → 180°
     *
     * @param angle The angle to normalize in degrees.
     * @return The normalized angle in [-180, 180] degrees.
     */
    public static double normalizeAngleDegrees(double angle) {
        // First bring to [0, 360) range
        angle = angle % 360.0;
        if (angle < 0) {
            angle += 360.0;
        }

        // Then shift to [-180, 180] range
        if (angle > 180.0) {
            angle -= 360.0;
        }

        return angle;
    }

    /**
     * Normalizes an angle to the range [-π, π] radians.
     * This is critical for swerve module control to ensure shortest-path rotation.
     * Uses modulo arithmetic for O(1) performance.
     *
     * Example (0→360° crossing):
     *   current = 350° (6.11 rad), target = 10° (0.17 rad)
     *   error = target - current = -5.94 rad
     *   normalized = 0.35 rad (≈20°) → servo turns +20° (correct!)
     *
     * @param angle The angle to normalize in radians.
     * @return The normalized angle in [-π, π] radians.
     */
    public static double normalizeAngleRadians(double angle) {
        // First bring to [0, 2π) range using modulo
        angle = angle % TWO_PI;
        if (angle < 0) {
            angle += TWO_PI;
        }

        // Then shift to [-π, π] range
        if (angle > Math.PI) {
            angle -= TWO_PI;
        }

        return angle;
    }

    /**
     * Calculates the shortest angular distance from 'from' to 'to'.
     * Result is in [-π, π] radians, representing the shortest path.
     *
     * @param from Starting angle in radians (any range)
     * @param to Target angle in radians (any range)
     * @return Shortest angular distance in [-π, π] radians
     */
    public static double shortestAngularDistance(double from, double to) {
        return normalizeAngleRadians(to - from);
    }

    /**
     * Normalizes angle to [0, 2π) range (0 to 360°).
     * Useful for display/telemetry where negative angles are confusing.
     *
     * @param angle The angle in radians
     * @return Normalized angle in [0, 2π) radians
     */
    public static double normalizeAngle0To2Pi(double angle) {
        angle = angle % TWO_PI;
        if (angle < 0) {
            angle += TWO_PI;
        }
        return angle;
    }

    /**
     * Wraps angle to [-180, 180] degrees.
     * Used for continuous angle tracking where we need signed representation.
     *
     * Examples:
     *   270° → -90°
     *   -270° → 90°
     *   370° → 10°
     *   -370° → -10°
     *
     * @param angleDegrees The angle to wrap in degrees
     * @return The wrapped angle in [-180, 180] degrees
     */
    public static double wrap180(double angleDegrees) {
        double wrapped = angleDegrees % 360.0;
        if (wrapped > 180.0) wrapped -= 360.0;
        if (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
    }
}