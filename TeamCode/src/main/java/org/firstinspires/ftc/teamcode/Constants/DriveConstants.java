package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

/**
 * Drive System Constants
 */
@Config
public class DriveConstants {

    // Robot dimensions (aprox, square robot)
    public static double WHEELBASE_METERS = 0.385;
    public static double TRACK_WIDTH_METERS = 0.385;

    // Module positions (+X forward, +Y left)
    public static final Translation2d FL_POSITION = new Translation2d(WHEELBASE_METERS/2, TRACK_WIDTH_METERS/2);
    public static final Translation2d FR_POSITION = new Translation2d(WHEELBASE_METERS/2, -TRACK_WIDTH_METERS/2);
    public static final Translation2d BL_POSITION = new Translation2d(-WHEELBASE_METERS/2, TRACK_WIDTH_METERS/2);
    public static final Translation2d BR_POSITION = new Translation2d(-WHEELBASE_METERS/2, -TRACK_WIDTH_METERS/2);

    // Drive motor config
    public static double WHEEL_DIAMETER_METERS = 0.0762;
    public static double DRIVE_GEAR_RATIO = 13.7;

    // Ticks/rev: GoBILDA 312=537.7, GoBILDA 435=384.5, REV HD=2800
    public static final double MOTOR_TICKS_PER_REV = 384.5;

    public static final double TICKS_PER_METER =
        (MOTOR_TICKS_PER_REV * DRIVE_GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_METERS);

    // Speed limits
    public static double MAX_SPEED_METERS_PER_SECOND = 3.0;
    public static double MAX_ANGULAR_VELOCITY = Math.PI;

    // Validation
    static {
        if (WHEEL_DIAMETER_METERS <= 0) {
            throw new IllegalStateException(
                "WHEEL_DIAMETER_METERS must be greater than 0. Current value: " + WHEEL_DIAMETER_METERS
            );
        }
        if (DRIVE_GEAR_RATIO <= 0) {
            throw new IllegalStateException(
                "DRIVE_GEAR_RATIO must be greater than 0. Current value: " + DRIVE_GEAR_RATIO
            );
        }
        if (MAX_SPEED_METERS_PER_SECOND <= 0) {
            throw new IllegalStateException(
                "MAX_SPEED_METERS_PER_SECOND must be greater than 0. Current value: " + MAX_SPEED_METERS_PER_SECOND
            );
        }
    }
}
