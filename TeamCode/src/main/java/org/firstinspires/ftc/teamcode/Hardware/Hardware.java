package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Subsystems.SwerveDrive;

/**
 * Central hardware manager - single initialization point for all robot components
 */
public class Hardware {
    public SwerveDrive swerveDrive;

    public SwerveHardware swerveHardware;
    public OuttakeHardware outtakeHardware;
    public IntakeHardware intakeHardware;

    public Hardware(HardwareMap hardwareMap) {
        swerveHardware = new SwerveHardware(hardwareMap);
        outtakeHardware = new OuttakeHardware(hardwareMap);
        intakeHardware = new IntakeHardware(hardwareMap);

        swerveDrive = new SwerveDrive(swerveHardware);
    }
}
