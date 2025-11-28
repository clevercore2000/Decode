package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * Steering constants for swerve modules
 * Uses voltage-based angle determination with 2:1 gear ratio
 */
@Config
public class SteeringConstants {
    // === GEAR RATIO ===
    public static final double SERVO_TO_WHEEL_RATIO = 2.0;  // Servo rotates 2x, wheel rotates 1x

    // === VOLTAGE CONFIGURATION ===
    public static final double MIN_VOLTAGE = 0.0;
    public static final double MAX_VOLTAGE = 3.3;

    // === CALIBRATION OFFSETS (Voltage when wheel points forward) ===
    public static final double FL_VOLTAGE_OFFSET = 0.843;
    public static final double FR_VOLTAGE_OFFSET = 2.1;
    public static final double BL_VOLTAGE_OFFSET = 2.4;
    public static final double BR_VOLTAGE_OFFSET = 1.6;

    // === PID GAINS (Phase 1: P-only, tune via FTC Dashboard) ===
    public static double STEER_P = 0.4;   // Proportional gain - start here, tune live
    public static double STEER_I = 0.0;   // Integral - keep disabled for position control
    public static double STEER_D = 0.0;   // Derivative - add in Phase 2 if needed

    // === CONTROL PARAMETERS ===
    public static double STEERING_DEADBAND_RADIANS = 0.02;  // ~1.15° tolerance to prevent jitter
    public static double MIN_SERVO_POWER = 0.01;            // Keep encoder electronics active
    public static double STATIC_FRICTION_COMPENSATION = 0.03; // Boost to overcome servo stiction

    // === FILTERING (Phase 3: Add if needed) ===
    public static double ENCODER_FILTER_ALPHA = 0;  // 0.0 = no filtering, 0.7-0.95 = EMA filtering
}
