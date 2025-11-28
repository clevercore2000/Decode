package org.firstinspires.ftc.teamcode.Utils;

public class MathUtils {

    /**
     * Normalizes an angle to the range [-180, 180] degrees.
     * @param angle The angle to normalize in degrees.
     * @return The normalized angle in degrees.
     */
    public static double normalizeAngle(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    /**
     * Normalizes an angle to the range [-π, π] radians.
     * This is critical for swerve module control to ensure shortest-path rotation.
     * @param angle The angle to normalize in radians.
     * @return The normalized angle in radians.
     */
    public static double normalizeAngleRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }
}