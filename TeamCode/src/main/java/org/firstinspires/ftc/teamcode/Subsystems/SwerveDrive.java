package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.SwerveHardware;

/**
 * Swerve Drive Subsystem
 *
 * Manages 4-module swerve drive using FTCLib WPILib kinematics
 * Converts chassis velocities (forward, strafe, rotation) to individual module states
 * FTCLib handles inverse kinematics
 */
public class SwerveDrive {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics kinematics;

    public SwerveDrive(SwerveHardware swerveHardware) {
        frontLeft = new SwerveModule(
            swerveHardware.flDrive,
            swerveHardware.flSteer,
            swerveHardware.flEncoder,
            SteeringConstants.FL_ANGLE_OFFSET,
            false,
            false,
            "FL"
        );

        frontRight = new SwerveModule(
            swerveHardware.frDrive,
            swerveHardware.frSteer,
            swerveHardware.frEncoder,
            SteeringConstants.FR_ANGLE_OFFSET,
            false,
            false,
            "FR"
        );

        backLeft = new SwerveModule(
            swerveHardware.blDrive,
            swerveHardware.blSteer,
            swerveHardware.blEncoder,
            SteeringConstants.BL_ANGLE_OFFSET,
            false,
            false,
            "BL"
        );

        backRight = new SwerveModule(
            swerveHardware.brDrive,
            swerveHardware.brSteer,
            swerveHardware.brEncoder,
            SteeringConstants.BR_ANGLE_OFFSET,
            false,
            false,
            "BR"
        );

        // Initialize kinematics with module positions
        kinematics = new SwerveDriveKinematics(
            DriveConstants.FL_POSITION,
            DriveConstants.FR_POSITION,
            DriveConstants.BL_POSITION,
            DriveConstants.BR_POSITION
        );
    }

    /**
     * Drive the robot with given velocities
     *
     * xSpeed Forward velocity in m/s (positive = forward)
     * ySpeed Strafe velocity in m/s (positive = left)
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize speeds if any exceed max
        SwerveDriveKinematics.normalizeWheelSpeeds(
            moduleStates,
            DriveConstants.MAX_SPEED_METERS_PER_SECOND
        );


        setModuleStates(moduleStates);
    }


    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (desiredStates.length != 4) {
            throw new IllegalArgumentException("Must provide exactly 4 module states");
        }

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }


    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }


    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }


    public void resetDriveEncoders() {
        frontLeft.resetDriveEncoder();
        frontRight.resetDriveEncoder();
        backLeft.resetDriveEncoder();
        backRight.resetDriveEncoder();
    }

    /**
     * Get a specific module by index (0=FL, 1=FR, 2=BL, 3=BR)
     */
    public SwerveModule getModule(int index) {
        switch (index) {
            case 0: return frontLeft;
            case 1: return frontRight;
            case 2: return backLeft;
            case 3: return backRight;
            default: throw new IllegalArgumentException("Invalid module index: " + index);
        }
    }
}
