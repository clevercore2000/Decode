package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
 * - Each SwerveModule uses PID control for steering (P=0.6, D=0.1)
 * - FTCLib handles inverse kinematics (ChassisSpeeds → SwerveModuleStates)
 * - Module ordering: Front-Left, Front-Right, Back-Left, Back-Right (CRITICAL!)
 */
public class SwerveDrive {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics kinematics;

    // Field-centric drive (requires IMU)
    private final IMU imu;

    public SwerveDrive(SwerveHardware swerveHardware) {
        // Field-centric drive
        imu = swerveHardware.imu;

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
     * Drive the robot with given velocities.
     *
     * This method performs the complete swerve drive control pipeline:
     * 1. Transforms velocities (field-centric → robot-centric if needed)
     * 2. Inverse kinematics (chassis velocity → module states)
     * 3. Speed normalization (ensures achievable speeds)
     * 4. Distributes commands to all 4 modules
     *
     * @param xSpeed Forward velocity in m/s (positive = forward, negative = backward)
     * @param ySpeed Strafe velocity in m/s (positive = left, negative = right)
     * @param rotSpeed Rotation velocity in rad/s (positive = CCW, negative = CW)
     * @param fieldRelative If true, velocities relative to field (requires IMU).
     *                      If false, velocities relative to robot orientation.
     *
     * Example:
     * <pre>
     *   drive(1.0, 0.5, 0.0, true);  // Move forward-left relative to field
     *   drive(0.0, 0.0, Math.PI, false);  // Rotate in place (robot-centric)
     * </pre>
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds;

        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, getGyroAngle()
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

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

    /****
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

    /**
     * Reset IMU heading to zero
     */
    public void zeroHeading() {
        imu.resetYaw();
    }

    /**
     * Get current IMU heading in degrees (for telemetry)
     */
    public double getHeadingDegrees() {
        return getGyroAngle().getDegrees();
    }

    /**
     * Get current robot heading as Rotation2d
     */
    private Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(
            imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );
    }

    // For odometry - update robot pose based on module states (not needed for basic teleop)
    /*
    public void updateOdometry() {
        // Implementation depends on odometry class setup
        // See FTCLib SwerveDriveOdometry documentation
    }
    */
}
