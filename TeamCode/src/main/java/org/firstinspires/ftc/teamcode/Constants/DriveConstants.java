package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Translation2d;

/**
 * Drive constants for swerve drivetrain
 * Contains robot geometry, motor specifications, and velocity limits
 */
@Config
public class DriveConstants {
    // === ROBOT GEOMETRY ===
    public static double WHEELBASE_METERS = 0.385;      // Front-to-back distance between modules
    public static double TRACK_WIDTH_METERS = 0.385;    // Left-to-right distance between modules

    // Module positions relative to robot center (FL, FR, BL, BR order - CRITICAL for kinematics)
    // +X = forward, +Y = left (FTCLib/WPILib convention)
    public static final Translation2d FL_POSITION = new Translation2d(0.1925, 0.1925);   // Front-left
    public static final Translation2d FR_POSITION = new Translation2d(0.1925, -0.1925);  // Front-right
    public static final Translation2d BL_POSITION = new Translation2d(-0.1925, 0.1925);  // Back-left
    public static final Translation2d BR_POSITION = new Translation2d(-0.1925, -0.1925); // Back-right

    // === WHEEL SPECIFICATIONS ===
    public static double WHEEL_DIAMETER_METERS = 0.0762;  // 3 inches = 0.0762m (76.2mm)
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;

    // === MOTOR SPECIFICATIONS ===
    public static double DRIVE_GEAR_RATIO = 13.7;             // Motor:Wheel gear reduction
    public static final double MOTOR_TICKS_PER_REV = 384.5;   // Encoder ticks per motor revolution
    public static final double TICKS_PER_METER =
            (MOTOR_TICKS_PER_REV * DRIVE_GEAR_RATIO) / WHEEL_CIRCUMFERENCE_METERS;

    // === VELOCITY LIMITS ===
    public static double MAX_SPEED_METERS_PER_SECOND = 4.0;   // Maximum translational speed
    public static double MAX_ANGULAR_VELOCITY = 4 * Math.PI;  // Maximum rotational speed (rad/s) - 2 rotations/sec

    // === DRIVE CONTROL (Simple feedforward for now, no velocity PID) ===
    public static double DRIVE_P = 0.0;   // Not needed for open-loop feedforward control
    public static double DRIVE_I = 0.0;   // Reserved for future velocity PID
    public static double DRIVE_D = 0.0;   // Reserved for future velocity PID
    public static double DRIVE_FF = 1.0 / MAX_SPEED_METERS_PER_SECOND;  // Feedforward gain
}
