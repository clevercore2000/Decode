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
    public static double MAX_DRIVE_SPEED = 3.0;        // m/s (matches DriveConstants.MAX_SPEED)
    public static double MAX_ROTATION_SPEED = Math.PI; // rad/s (matches DriveConstants.MAX_ANGULAR_VELOCITY)

}
