package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SteeringConstants {

    // Axon Mini: 2:1 reduction (0-3.3V = 0-720° servo = 0-360° wheel)
    public static final double SERVO_TO_STEERING_RATIO = 0.5;
    public static final double ANALOG_VOLTAGE_MAX = 3.3;

    // Module angle offsets - run SwerveCalibration to recalibrate
    public static double FL_ANGLE_OFFSET = -2.24;
    public static double FR_ANGLE_OFFSET = 2.83;
    public static double BL_ANGLE_OFFSET = 2.4;
    public static double BR_ANGLE_OFFSET = 2.01;

    // Steering control (Axon has internal PID, start ~0.05, max ~0.2)
    public static double STEER_KP = 0.05;

    // Encoder filtering (0.95 = 95% new, 5% old)
    public static double ENCODER_FILTER_ALPHA = 0.95;

    // Deadband (~2.86°) - prevents jitter near target
    public static double STEERING_DEADBAND_RADIANS = 0.05;
}
