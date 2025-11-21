package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

/**
 * Drive System Constants
 * Configuration for drive motors, robot dimensions, and speed limits
 */
@Config
public class DriveConstants {

    // ========== ROBOT DIMENSIONS ==========
    // Wheelbase: front-to-back distance between module centers (meters)
    public static double WHEELBASE_METERS = 0.325;

    // Track width: left-to-right distance between module centers (meters)
    public static double TRACK_WIDTH_METERS = 0.445;

    // Module positions relative to robot center (+X forward, +Y left)
    public static final Translation2d FL_POSITION =
        new Translation2d(WHEELBASE_METERS/2, TRACK_WIDTH_METERS/2);
    public static final Translation2d FR_POSITION =
        new Translation2d(WHEELBASE_METERS/2, -TRACK_WIDTH_METERS/2);
    public static final Translation2d BL_POSITION =
        new Translation2d(-WHEELBASE_METERS/2, TRACK_WIDTH_METERS/2);
    public static final Translation2d BR_POSITION =
        new Translation2d(-WHEELBASE_METERS/2, -TRACK_WIDTH_METERS/2);

    // ========== DRIVE MOTOR CONFIGURATION ==========
    public static double WHEEL_DIAMETER_METERS = 0.0762;  // Wheel diameter

    public static double DRIVE_GEAR_RATIO = 13.7;  // Motor rotations per wheel rotation

    /**
     * Motor encoder ticks per revolution
     * GoBILDA 312 RPM: 537.7, GoBILDA 435 RPM: 384.5, REV HD Hex: 2800
     */
    public static final double MOTOR_TICKS_PER_REV = 384.5;

    // Auto-calculated ticks per meter
    public static final double TICKS_PER_METER =
        (MOTOR_TICKS_PER_REV * DRIVE_GEAR_RATIO) / (Math.PI * WHEEL_DIAMETER_METERS);

    // ========== SPEED LIMITS ==========
    public static double MAX_SPEED_METERS_PER_SECOND = 3.0;  // Maximum linear speed (m/s)

    public static double MAX_ANGULAR_VELOCITY = Math.PI;  // Maximum rotation speed (rad/s)

    // ========== DRIVE PID CONSTANTS ==========
    /**
     * Drive velocity PID gains
     * Used for precise speed control of drive motors
     */
    public static double DRIVE_P = 0.1;
    public static double DRIVE_I = 0.0;
    public static double DRIVE_D = 0.0;

    /**
     * Drive feedforward gain
     * Maps desired velocity to motor power
     * Calculated as 1 / max_velocity
     */
    public static double DRIVE_FF = 1.0 / MAX_SPEED_METERS_PER_SECOND;

    // ========== CONFIGURATION VALIDATION ==========
    // Static initializer to validate configuration values at class loading
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
