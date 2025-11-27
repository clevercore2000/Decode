package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * Drive System Constants
 * Configuration for drive motors, robot dimensions, and speed limits
 */
@Config
public class DriveConstants {

    // ========== ROBOT DIMENSIONS ==========
    // SQUARE ROBOT: All modules equidistant from center
    // Wheelbase: front-to-back distance between module centers (meters)
    // Track width: left-to-right distance between module centers (meters)
    // IMPORTANT: For square robot, these MUST be equal!
    public static double WHEELBASE_METERS = 0.385;
    public static double TRACK_WIDTH_METERS = 0.385;  // Same as wheelbase (square config)

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

    // ========== DRIVE CONTROL ==========
    /**
     * Drive motor control uses simple feedforward (no velocity PID)
     * motorPower = targetSpeed (already normalized by kinematics)
     *
     * This works because:
     * - Motors naturally reach commanded speed at given power
     * - Custom kinematics handles speed normalization
     * - Simple and reliable for teleop
     */

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
