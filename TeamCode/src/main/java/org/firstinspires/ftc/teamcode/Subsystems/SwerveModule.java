package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.SwerveModuleState;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.DriveConstants;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;

/**
 * Swerve module implementation with PID steering control
 * Based on proven KookyBotz approach - external PID DOES work with Axon servos!
 *
 * Key features:
 * - PID controller (P=0.6, D=0.1) for smooth, responsive steering
 * - Static friction compensation to overcome servo stiction
 * - Servos kept powered to maintain analog encoder readings
 * - State optimization to avoid >90° rotations
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

    // PID controller for steering
    private final PIDFController rotationController;

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

        // Initialize PID controller for steering
        this.rotationController = new PIDFController(
            SteeringConstants.STEER_P,
            SteeringConstants.STEER_I,
            SteeringConstants.STEER_D,
            0  // F term not used
        );

        // Initialize filtered angle
        this.filteredAngle = getSteeringAngle();
    }

    /**
     * Set the desired state for this module.
     *
     * This method performs the complete module control pipeline:
     * 1. State optimization (avoids >90° rotation by reversing drive instead)
     * 2. PID steering control (P=0.6, D=0.1) with static friction compensation
     * 3. Minimum power enforcement (keeps encoder electronics active)
     * 4. Feedforward drive control (open-loop speed command)
     *
     * @param desiredState Target state containing:
     *                     - speedMetersPerSecond: Drive velocity
     *                     - angle: Target steering angle (Rotation2d)
     *
     * Note: State is automatically optimized - if target angle requires >90° rotation,
     * module will rotate ≤90° and reverse drive motor direction instead.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize state to avoid rotating more than 90 degrees
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            desiredState,
            new Rotation2d(getSteeringAngle())
        );

        this.desiredState = optimizedState;

        // === STEERING CONTROL (PID WITH STATIC FRICTION) ===
        double currentAngle = getSteeringAngle();
        double targetAngle = optimizedState.angle.getRadians();

        // Calculate shortest angle error (wrap at 2π)
        double angleError = normalizeAngle(targetAngle - currentAngle);

        // Update PID gains (allows FTC Dashboard tuning)
        rotationController.setPIDF(
            SteeringConstants.STEER_P,
            SteeringConstants.STEER_I,
            SteeringConstants.STEER_D,
            0  // F term not used for steering
        );

        // PID control (setpoint = 0, measurement = error)
        double steerPower = Range.clip(
            rotationController.calculate(0, angleError),
            -SteeringConstants.MAX_SERVO_POWER,
            SteeringConstants.MAX_SERVO_POWER
        );

        // Safety: handle NaN (can occur during rapid changes)
        if (Double.isNaN(steerPower)) {
            steerPower = 0;
        }

        // Add static friction compensation when error is significant
        if (Math.abs(angleError) > SteeringConstants.ERROR_THRESHOLD_FOR_STATIC) {
            steerPower += SteeringConstants.K_STATIC * Math.signum(steerPower);
        }

        // CRITICAL: Keep servo powered even when at target to maintain encoder readings!
        // Axon encoders require PWM signals active to output voltage
        if (Math.abs(steerPower) < SteeringConstants.MIN_SERVO_POWER) {
            steerPower = SteeringConstants.MIN_SERVO_POWER * Math.signum(angleError);
        }

        // Apply power (with inversion if needed)
        steerServo.setPower(steerInverted ? -steerPower : steerPower);

        // === DRIVE CONTROL (SIMPLE FEEDFORWARD) ===
        double targetSpeed = optimizedState.speedMetersPerSecond;

        // OPTIONAL: Cosine compensation (currently disabled)
        // Reduces wheel slip/skew when module not yet pointed at target angle
        // Formula: targetSpeed *= Math.abs(Math.cos(angleError))
        // Effect: If module 45° off target, scale speed by cos(45°) ≈ 0.707
        //
        // When to enable:
        // - High-speed autonomous with rapid direction changes
        // - Competition on slippery surfaces (reduce wheel scrubbing)
        // - After basic functionality verified (adds complexity)
        //
        // When to leave disabled:
        // - Teleop (driver compensates naturally)
        // - Good traction surfaces (scrubbing minimal)
        // - Tuning/debugging (one less variable)

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
