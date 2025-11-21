package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * Steering System Constants
 * Configuration for Axon Mini servos, analog encoders, and steering PID
 */
@Config
public class SteeringConstants {

    // ========== AXON MINI SERVO CONFIGURATION ==========
    // Gear ratio from servo to steering axis (2:1 reduction)
    // Encoder on servo shaft: 0-3.3V = 0-720° servo = 0-360° wheel
    // Multiply servo angle by 0.5 to get wheel angle
    public static final double SERVO_TO_STEERING_RATIO = 0.5;

    /**
     * Maximum analog voltage from Axon encoder
     * Standard analog range is 0-3.3V
     */
    public static final double ANALOG_VOLTAGE_MAX = 3.3;

    // ========== MODULE ANGLE OFFSETS ==========
    // IMPORTANT: These offsets are INVALID after fixing gear ratio bug!
    // Run SwerveCalibration OpMode to get new values
    public static double FL_ANGLE_OFFSET = -2.24;  // TODO: Recalibrate
    public static double FR_ANGLE_OFFSET = -3.103;  // TODO: Recalibrate
    public static double BL_ANGLE_OFFSET = 2.1;  // TODO: Recalibrate
    public static double BR_ANGLE_OFFSET = 1.9;  // TODO: Recalibrate

    // ========== STEERING PID CONSTANTS ==========
    // Reduced to fix jittering/oscillation (servos shaking rapidly)
    public static double STEER_P = 0.25;  // Lowered from 0.6 to reduce jitter
    public static double STEER_I = 0.0;
    public static double STEER_D = 0.05;  // Lowered from 0.1

    // ========== ENCODER FILTERING ==========
    // EMA filter alpha (0.0-1.0, higher = less filtering)
    // TODO: tune if needed
    public static double ENCODER_FILTER_ALPHA = 0.7;

    // Steering deadband in radians (~1.15 degrees)
    // TODO: tune if needed
    public static double STEERING_DEADBAND_RADIANS = 0.02;
}
