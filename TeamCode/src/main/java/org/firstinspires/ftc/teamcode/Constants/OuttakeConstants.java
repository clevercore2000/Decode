package org.firstinspires.ftc.teamcode.Constants;

/**
 * Constants for Outtake Subsystem (2-motor wheel mechanism)
 * Defines PID gains and target velocities for RPM control
 */
public class OuttakeConstants {

    // =============== RAMP SERVO CONFIG & POSITIONS ===============
    // Range: 0-355° for Axon Mini servo (full range, 5° dead zone at 355-360)

    public static final double RAMP_MAX = 355;  // Axon Mini maximum angle
    public static final double RAMP_MIN = 0;    // Axon Mini minimum angle

    public static final double RAMP_IDLE = 92.5;   // Measured using ServoTest

    public static final double RAMP_SHOOT = 127.5; // Measured using ServoTest






    // ========== MOTOR SPECIFICATIONS ==========
    // Maximum motor RPM (GoBILDA Yellow Jacket = 5281, REV HD Hex = 6000)
    public static final double MAX_MOTOR_RPM = 6000.0;

    /**
     * Encoder ticks per motor revolution
     * Configure based on motor specs:
     * - GoBILDA Yellow Jacket (312 RPM): 537.7 ticks/rev
     * - REV HD Hex Motor: 28 ticks/rev
     * - REV Core Hex Motor: 288 ticks/rev
     */
    public static final double MOTOR_TICKS_PER_REV = 28.0;

    // ========== TARGET VELOCITIES ==========
    // Target outtake RPM
    public static final double TARGET_RPM = 2700.0;

    // Target velocity in ticks per second
    //public static final double TARGET_VELOCITY_TPS = (TARGET_RPM / 60.0) * f;

    // ========== PID GAINS ==========
    public static final double VELOCITY_P = 0.0002;  // Proportional gain
    public static final double VELOCITY_I = 0.00001; // Integral gain
    public static final double VELOCITY_D = 0.00001; // Derivative gain

    // Feedforward (auto-calculated from motor specs)
    public static final double VELOCITY_FF = 1.0 / ((MAX_MOTOR_RPM / 60.0) * MOTOR_TICKS_PER_REV);

    public static final double MAX_INTEGRAL = 0.1;   // Integral windup limit

    // ========== CONTROL PARAMETERS ==========
    public static final double RPM_TOLERANCE = 15.0; // Acceptable error for "at target" check
    public static final double MIN_POWER = 0.05;     // Minimum power to overcome friction
}