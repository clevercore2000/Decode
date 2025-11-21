package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.kinematics.wpilibkinematics.ChassisSpeeds;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveDriveKinematics;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Hardware.SwerveHardware;

/**
 * Swerve Drive Subsystem
 * Converts chassis velocities to module states using WPILib kinematics
 */
public class SwerveDrive {

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics kinematics;

    //TODO Uncomment for field-centric drive
    // private final IMU imu;

    public SwerveDrive(SwerveHardware swerveHardware) {
        //TODO Uncomment for field-centric drive
        // imu = swerveHardware.imu;

        // Note: For coaxial swerve, all drive motors should have the same inversion
        // based on physical mounting orientation, NOT left/right side
        // Adjust all to true if motors run backwards when given positive power
        frontLeft = new SwerveModule(
            swerveHardware.flDrive,
            swerveHardware.flSteer,
            swerveHardware.flEncoder,
            SteeringConstants.FL_ANGLE_OFFSET,
            false,  // driveInverted
            false,  // steerInverted
            "FL"
        );

        frontRight = new SwerveModule(
            swerveHardware.frDrive,
            swerveHardware.frSteer,
            swerveHardware.frEncoder,
            SteeringConstants.FR_ANGLE_OFFSET,
            false,  // driveInverted - changed from true (tank drive logic)
            false,  // steerInverted
            "FR"
        );

        backLeft = new SwerveModule(
            swerveHardware.blDrive,
            swerveHardware.blSteer,
            swerveHardware.blEncoder,
            SteeringConstants.BL_ANGLE_OFFSET,
            false,  // driveInverted
            false,  // steerInverted
            "BL"
        );

        backRight = new SwerveModule(
            swerveHardware.brDrive,
            swerveHardware.brSteer,
            swerveHardware.brEncoder,
            SteeringConstants.BR_ANGLE_OFFSET,
            false,  // driveInverted - changed from true (tank drive logic)
            false,  // steerInverted
            "BR"
        );

        kinematics = new SwerveDriveKinematics(
            DriveConstants.FL_POSITION,
            DriveConstants.FR_POSITION,
            DriveConstants.BL_POSITION,
            DriveConstants.BR_POSITION
        );
    }

    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

        //TODO Uncomment for field-centric drive
        /*
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, rotSpeed, getGyroAngle()
            );
        } else {
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }
        */

        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

        // Normalize to stay within speed limits while preserving motion ratios
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

    //TODO Uncomment for field-centric drive
    /*
    public void zeroHeading() {
        imu.resetYaw();
    }

    private Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(
            imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)
        );
    }
    */

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
