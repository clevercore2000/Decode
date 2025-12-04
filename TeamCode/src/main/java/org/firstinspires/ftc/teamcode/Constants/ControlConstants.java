package org.firstinspires.ftc.teamcode.Constants;

import com.acmerobotics.dashboard.config.Config;

/**
 * Control constants for teleop/driver control
 * Includes joystick deadbands and speed scaling
 */
@Config
public class ControlConstants {
    // === JOYSTICK CONFIGURATION ===
    public static double JOYSTICK_DEADBAND = 0.05;  // Ignore inputs below 5% to prevent drift

    // === SPEED SCALING ===
    public static double MAX_DRIVE_SPEED = DriveConstants.MAX_SPEED_METERS_PER_SECOND;  // Single source of truth
    public static double MAX_ROTATION_SPEED = 4 * Math.PI; // rad/s - 2 rotations/sec (typical FTC competitive speed)

}
