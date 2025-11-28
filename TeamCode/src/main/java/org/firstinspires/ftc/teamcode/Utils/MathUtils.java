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
}
