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
 *
 * Architecture:
 * - Uses simple proportional control for steering (trust Axon servo internal PID)
 * - FTCLib handles inverse kinematics (ChassisSpeeds → SwerveModuleStates)
 * - Module ordering: Front-Left, Front-Right, Back-Left, Back-Right
 */
public class SwerveDrive {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics kinematics;

    // Uncomment for field-centric drive (requires IMU)
    // private final IMU imu;

    public SwerveDrive(SwerveHardware swerveHardware) {
        // Uncomment for field-centric drive
        // imu = swerveHardware.imu;

        // Initialize modules (FL, FR, BL, BR order is important - matches kinematics)
        frontLeft = new SwerveModule(
            swerveHardware.flDrive,
            swerveHardware.flSteer,
            swerveHardware.flEncoder,
            SteeringConstants.FL_ANGLE_OFFSET,
            false,  // driveInverted - adjust if motors run backwards
            false,  // steerInverted
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
     * @param xSpeed Forward velocity in m/s (positive = forward)
     * @param ySpeed Strafe velocity in m/s (positive = left)
     * @param rotSpeed Rotation velocity in rad/s (positive = CCW)
     * @param fieldRelative Whether velocities are field-relative (requires IMU)
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds;

        // Uncomment for field-centric drive (requires IMU initialized)
        /*
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, getGyroAngle()
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }
        */

        // Robot-centric drive (default)
        chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

        // Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize speeds if any exceed max (preserves motion ratios)
        SwerveDriveKinematics.normalizeWheelSpeeds(
            moduleStates,
            DriveConstants.MAX_SPEED_METERS_PER_SECOND
        );

        // Send states to modules
        setModuleStates(moduleStates);
    }

    /**
     * Set desired states for all modules
     * Order must match kinematics: FL, FR, BL, BR
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        if (desiredStates.length != 4) {
            throw new IllegalArgumentException("Must provide exactly 4 module states");
        }

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    /**
     * Get current states of all modules (for odometry/telemetry)
     * Returns in order: FL, FR, BL, BR
     */
    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        };
    }

    /**
     * Stop all modules (zero power)
     */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /**
     * Reset all drive encoders to zero
     */
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

    // Uncomment for field-centric drive
    /*
    public void zeroHeading() {
        imu.resetYaw();
    }

    private Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(
            imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );
    }

    // For odometry - update robot pose based on module states
    public void updateOdometry() {
        // Implementation depends on odometry class setup
        // See FTCLib SwerveDriveOdometry documentation
    }
    */
}
