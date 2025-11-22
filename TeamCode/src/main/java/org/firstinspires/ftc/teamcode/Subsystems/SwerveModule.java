package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;

/**
 * Simple, proven swerve module implementation
 * Uses basic proportional control for steering - lets Axon internal PID do the work
 *
 * Key changes from complex version:
 * - NO external PID controller (Axon servo has internal PID)
 * - Simple proportional control: power = error × gain
 * - Minimal filtering to reduce lag
 * - Trust the hardware, keep software simple
 */
public class SwerveModule {

    // Hardware
    private final DcMotorEx driveMotor;
    private final CRServo steerServo;
    private final AnalogInput steerEncoder;

    // Configuration
    private final double angleOffset;
    private final boolean steerInverted;
    private final String moduleName;

    // Minimal filtering for encoder noise
    private double filteredAngle = 0;

    // State tracking
    private SwerveModuleState desiredState = new SwerveModuleState();

    public SwerveModule(
        DcMotorEx driveMotor,
        CRServo steerServo,
        AnalogInput steerEncoder,
        double angleOffset,
        boolean driveInverted,
        boolean steerInverted,
        String moduleName
    ) {
        this.driveMotor = driveMotor;
        this.steerServo = steerServo;
        this.steerEncoder = steerEncoder;
        this.angleOffset = angleOffset;
        this.steerInverted = steerInverted;
        this.moduleName = moduleName;

        // Configure drive motor
        this.driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (driveInverted) {
            this.driveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        // Initialize filtered angle
        this.filteredAngle = getSteeringAngle();
    }

    /**
     * Set the desired state for this module
     * Uses simple proportional control - proven to work
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize state to avoid rotating more than 90 degrees
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            desiredState,
            new Rotation2d(getSteeringAngle())
        );

        this.desiredState = optimizedState;

        // === STEERING CONTROL (SIMPLE PROPORTIONAL) ===
        double currentAngle = getSteeringAngle();
        double targetAngle = optimizedState.angle.getRadians();

        // Calculate shortest angle error (wrap at 2π)
        double angleError = normalizeAngle(targetAngle - currentAngle);

        // Simple proportional control - let Axon servo internal PID do the heavy lifting
        double steerPower;
        if (Math.abs(angleError) < SteeringConstants.STEERING_DEADBAND_RADIANS) {
            // Within deadband - stop to prevent jitter
            steerPower = 0.0;
        } else {
            // Simple: power = error × gain
            steerPower = angleError * SteeringConstants.STEER_KP;
            // Clamp to valid servo range
            steerPower = Math.max(-1.0, Math.min(1.0, steerPower));
        }

        steerServo.setPower(steerInverted ? -steerPower : steerPower);

        // === DRIVE CONTROL (SIMPLE FEEDFORWARD) ===
        // Cosine compensation reduces skew during direction changes
        // TEMPORARILY DISABLED for testing - re-enable after motors work
        // double cosineScalar = Math.abs(Math.cos(angleError));
        // double targetSpeed = optimizedState.speedMetersPerSecond * cosineScalar;
        double targetSpeed = optimizedState.speedMetersPerSecond;

        // Kinematics already normalizes speeds to max - just clamp to [-1, 1]
        // NO additional division needed!
        double drivePower = Math.max(-1.0, Math.min(1.0, targetSpeed));

        driveMotor.setPower(drivePower);
    }

    /**
     * Get the current state of the module
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            getDriveVelocity(),
            new Rotation2d(getSteeringAngle())
        );
    }

    /**
     * Reset the drive encoder to zero
     */
    public void resetDriveEncoder() {
        driveMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Get the current steering angle in radians
     * Minimal filtering to reduce lag
     */
    private double getSteeringAngle() {
        // Read analog voltage (0-3.3V = 0-355° with ~5° dead zone)
        double rawVoltage = steerEncoder.getVoltage();

        // Convert voltage to angle (0-3.3V = 0-2π radians)
        double rawAngle = (rawVoltage / SteeringConstants.ANALOG_VOLTAGE_MAX) * 2 * Math.PI;

        // Minimal EMA filtering (95% new, 5% old) - barely any lag
        filteredAngle = SteeringConstants.ENCODER_FILTER_ALPHA * rawAngle
                      + (1 - SteeringConstants.ENCODER_FILTER_ALPHA) * filteredAngle;

        // Apply calibration offset and normalize
        double angle = filteredAngle - angleOffset;
        return normalizeAngle(angle);
    }

    /**
     * Get raw encoder voltage for debugging
     */
    public double getRawEncoderVoltage() {
        return steerEncoder.getVoltage();
    }

    /**
     * Get the drive velocity in meters per second
     */
    private double getDriveVelocity() {
        // Get velocity in ticks per second
        double velocityTicksPerSec = driveMotor.getVelocity();

        // Convert to meters per second
        return velocityTicksPerSec / DriveConstants.TICKS_PER_METER;
    }

    /**
     * Normalize angle to [-π, π] range
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Get the module name for telemetry
     */
    public String getModuleName() {
        return moduleName;
    }

    /**
     * Get the desired state (for telemetry/debugging)
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Get current steering angle for telemetry
     */
    public double getCurrentSteeringAngle() {
        return getSteeringAngle();
    }

    /**
     * Get current drive motor power for debugging
     */
    public double getDrivePower() {
        return driveMotor.getPower();
    }

    /**
     * Stop the module (zero power to drive and steer)
     */
    public void stop() {
        driveMotor.setPower(0.0);
        steerServo.setPower(0.0);
    }
}
