package org.firstinspires.ftc.teamcode.Hardware;

import android.database.AbstractCursor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

/**
 * Central hardware manager for DECODE season robot
 * Single initialization point for all robot components
 *
 * Architecture:
 * - Instantiates hardware abstraction classes (e.g., SwerveHardware)
 * - Passes hardware to subsystems (e.g., SwerveDrive)
 * - Provides single access point for all robot systems
 *
 * Current Configuration:
 * - Swerve Drive: 4-module coaxial swerve with Axon Mini servos
 */
public class Hardware {
    // Subsystems
    public SwerveDrive swerveDrive;

    // Hardware abstraction layers
    public SwerveHardware swerveHardware;
    public OuttakeHardware outtakeHardware;
    public IntakeHardware intakeHardware;

    public Hardware(HardwareMap hardwareMap) {
        // Initialize hardware abstraction layers
        swerveHardware = new SwerveHardware(hardwareMap);
        outtakeHardware = new OuttakeHardware(hardwareMap);
        intakeHardware = new IntakeHardware(hardwareMap);

        // Initialize subsystems with their hardware
        swerveDrive = new SwerveDrive(swerveHardware);
    }
}
