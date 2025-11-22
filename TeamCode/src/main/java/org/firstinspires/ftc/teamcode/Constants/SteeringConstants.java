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
    public static double FR_ANGLE_OFFSET = 2.83;  // TODO: Recalibrate
    public static double BL_ANGLE_OFFSET = 2.4;  // TODO: Recalibrate
    public static double BR_ANGLE_OFFSET = 2.01;  // TODO: Recalibrate

    // ========== STEERING CONTROL CONSTANTS ==========
    // Simple proportional gain for steering (NO PID!)
    // Axon servo has internal PID - we just give it direction and let it work
    // Start LOW (0.05) and increase slowly if needed (max ~0.2)
    public static double STEER_KP = 0.05;

    // ========== ENCODER FILTERING ==========
    // Minimal filtering (0.95 = 95% new reading, 5% old)
    // Reduces lag while still smoothing encoder noise
    public static double ENCODER_FILTER_ALPHA = 0.95;

    // Steering deadband in radians (~2.86 degrees)
    // Larger deadband prevents jitter when close to target
    public static double STEERING_DEADBAND_RADIANS = 0.05;
}
