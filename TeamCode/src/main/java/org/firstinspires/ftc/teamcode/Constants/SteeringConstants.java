package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * Steering System Constants
 * Configuration for Axon Mini servos, analog encoders, and steering PID
 */
@Config
public class SteeringConstants {

    // ========== AXON MINI SERVO CONFIGURATION ==========
    /**
     * Maximum analog voltage from Axon encoder
     * Standard analog range is 0-3.3V
     *
     * NOTE: Encoder reads wheel angle DIRECTLY (not servo angle)
     * 0-3.3V = 0-360° wheel rotation (encoder is after 2:1 gear reduction)
     * No gear ratio correction needed in software!
     */
    public static final double ANALOG_VOLTAGE_MAX = 3.3;

    // ========== MODULE ANGLE OFFSETS ==========
    // IMPORTANT: These offsets are INVALID after fixing gear ratio bug!
    // Run SwerveCalibration OpMode to get new values
    public static double FL_ANGLE_OFFSET = -2.24;  // TODO: Recalibrate
    public static double FR_ANGLE_OFFSET = 2.83;  // TODO: Recalibrate
    public static double BL_ANGLE_OFFSET = 2.4;  // TODO: Recalibrate
    public static double BR_ANGLE_OFFSET = 2.01;  // TODO: Recalibrate

    // ========== STEERING CONTROL CONSTANTS ==========
    // PID gains for steering (based on proven KookyBotz implementation)
    // NOTE: External PID DOES work with Axon servos! Previous diagnosis was wrong.
    // Issue was gain too low (0.05), not PID conflict.
    public static double STEER_P = 0.6;   // Proportional gain (was 0.05 - 12× too low!)
    public static double STEER_I = 0.0;   // Integral gain (not needed for position control)
    public static double STEER_D = 0.1;   // Derivative gain (dampens oscillations)

    // Static friction compensation (helps overcome servo stiction)
    // Added when error > threshold to help servo start moving
    public static double K_STATIC = 0.03;
    public static double ERROR_THRESHOLD_FOR_STATIC = 0.02;  // radians (~1.15 degrees)

    // Maximum servo power limit
    public static double MAX_SERVO_POWER = 1.0;

    // Minimum power to keep servo electronics active (CRITICAL for encoder reading!)
    // Axon encoders require servo PWM active to output voltage
    public static double MIN_SERVO_POWER = 0.01;

    // ========== ENCODER FILTERING ==========
    // Minimal filtering (0.95 = 95% new reading, 5% old)
    // Reduces lag while still smoothing encoder noise
    public static double ENCODER_FILTER_ALPHA = 0.95;

    // ========== ENCODER DEAD ZONE ==========
    /**
     * Encoder dead zone near 360° where voltage wraps
     * Approximately 355-360° (5° gap where voltage jumps from 3.3V to 0V)
     */
    public static final double ENCODER_DEAD_ZONE_START_DEGREES = 355.0;
    public static final double ENCODER_DEAD_ZONE_START_RADIANS = Math.toRadians(355.0);
}
