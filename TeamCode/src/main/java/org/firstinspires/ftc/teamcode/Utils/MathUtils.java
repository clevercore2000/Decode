package org.firstinspires.ftc.teamcode.Utils;

public class MathUtils {
    private static final double TWO_PI = 2.0 * Math.PI;

    /** Normalizes angle to [-180, 180] degrees */
    public static double normalizeAngleDegrees(double angle) {
        angle = angle % 360.0;
        if (angle < 0) angle += 360.0;
        if (angle > 180.0) angle -= 360.0;
        return angle;
    }

    /** Normalizes angle to [-π, π] radians */
    public static double normalizeAngleRadians(double angle) {
        angle = angle % TWO_PI;
        if (angle < 0) angle += TWO_PI;
        if (angle > Math.PI) angle -= TWO_PI;
        return angle;
    }

    /** Shortest angular distance from 'from' to 'to' in [-π, π] radians */
    public static double shortestAngularDistance(double from, double to) {
        return normalizeAngleRadians(to - from);
    }

    /** Normalizes angle to [0, 2π) radians */
    public static double normalizeAngle0To2Pi(double angle) {
        angle = angle % TWO_PI;
        if (angle < 0) angle += TWO_PI;
        return angle;
    }

    /** Wraps angle to [-180, 180] degrees */
    public static double wrap180(double angleDegrees) {
        double wrapped = angleDegrees % 360.0;
        if (wrapped > 180.0) wrapped -= 360.0;
        if (wrapped < -180.0) wrapped += 360.0;
        return wrapped;
    }
}
