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
import org.firstinspires.ftc.teamcode.Subsystems.Utils.EMAFilter;

/**
 * Represents a single swerve module with coaxial design
 * Controls one drive motor and one steering servo (Axon Mini with analog feedback)
 *
 * Responsibilities:
 * - Read steering position from analog encoder
 * - Control steering angle using CRServo with custom PID
 * - Control drive velocity
 * - Optimize module states (never rotate >90°)
 * - Provide current state for telemetry
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

    // Controllers
    private final PIDController steerPID;
    private final PIDController drivePID;

    // Encoder filtering
    private final EMAFilter encoderFilter;

    // State tracking
    private SwerveModuleState desiredState = new SwerveModuleState();

    /**
     * Create a new swerve module
     *
     * @param driveMotor Motor for driving the wheel (must be DcMotorEx for velocity control)
     * @param steerServo CRServo for steering (Axon Mini)
     * @param steerEncoder Analog input for steering position
     * @param angleOffset Calibration offset in radians
     * @param driveInverted Whether to invert drive motor direction
     * @param steerInverted Whether to invert steering servo direction
     * @param moduleName Name for telemetry (e.g., "FL")
     */
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

        // Initialize PID controllers
        this.steerPID = new PIDController(
            SteeringConstants.STEER_P,
            SteeringConstants.STEER_I,
            SteeringConstants.STEER_D
        );
        // Enable continuous input for steering (angles wrap at 2π)
        this.steerPID.enableContinuousInput(-Math.PI, Math.PI);

        this.drivePID = new PIDController(
            DriveConstants.DRIVE_P,
            DriveConstants.DRIVE_I,
            DriveConstants.DRIVE_D
        );

        // Initialize encoder filter to reduce noise
        this.encoderFilter = new EMAFilter();
    }

    /**
     * Set the desired state for this module
     * Optimizes the state to minimize rotation and applies control
     *
     * @param desiredState Target speed and angle
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize state to avoid rotating more than 90 degrees
        SwerveModuleState optimizedState = SwerveModuleState.optimize(
            desiredState,
            new Rotation2d(getSteeringAngle())
        );

        this.desiredState = optimizedState;

        // Steering angle PID control
        double currentAngle = getSteeringAngle();
        double targetAngle = optimizedState.angle.getRadians();

        // Normalize angle error (wraps at 2π)
        double angleError = targetAngle - currentAngle;
        angleError = normalizeAngle(angleError);
        double normalizedTarget = currentAngle + angleError;

        // Apply deadband to reduce servo jitter
        double steerPower;
        if (Math.abs(angleError) < SteeringConstants.STEERING_DEADBAND_RADIANS) {
            steerPower = 0.0;
        } else {
            steerPower = steerPID.calculate(normalizedTarget, currentAngle);
            steerPower = Math.max(-1.0, Math.min(1.0, steerPower));
        }

        steerServo.setPower(steerInverted ? -steerPower : steerPower);

        // Drive motor PID + feedforward with cosine compensation
        // Cosine compensation reduces skew during direction changes (WPILib pattern)
        double cosineScalar = Math.abs(Math.cos(angleError));
        double currentVelocity = getDriveVelocity();
        double targetVelocity = optimizedState.speedMetersPerSecond * cosineScalar;

        double driveFeedforward = targetVelocity * DriveConstants.DRIVE_FF;
        double driveFeedback = drivePID.calculate(targetVelocity, currentVelocity);
        double drivePower = Math.max(-1.0, Math.min(1.0, driveFeedforward + driveFeedback));

        driveMotor.setPower(drivePower);
    }

    /**
     * Get the current state of the module (velocity and angle)
     *
     * @return Current module state
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
     * Reads analog encoder, applies gear ratio and offset
     *
     * @return Steering angle in radians [-π, π]
     */
    private double getSteeringAngle() {
        // Read analog voltage (0-3.3V) and convert to angle
        // Axon Mini: encoder on servo shaft with 2:1 reduction
        // 0-3.3V = 0-720° servo rotation = 0-360° wheel rotation
        double rawVoltage = steerEncoder.getVoltage();

        // Apply EMA filter to reduce noise
        double filteredVoltage = encoderFilter.filter(rawVoltage, SteeringConstants.ENCODER_FILTER_ALPHA);

        // Convert voltage to wheel angle
        // 0-3.3V = 0-355° wheel rotation (encoder measures wheel directly, not servo)
        // Note: ~5° dead zone exists where voltage jumps from 3.3V to 0V
        double rawAngle = (filteredVoltage / SteeringConstants.ANALOG_VOLTAGE_MAX) * 2 * Math.PI;

        // Apply calibration offset and normalize
        double angle = rawAngle - angleOffset;
        return normalizeAngle(angle);
    }

    /**
     * Get raw encoder voltage for debugging
     *
     * @return Encoder voltage (0-3.3V)
     */
    public double getRawEncoderVoltage() {
        return steerEncoder.getVoltage();
    }

    /**
     * Get the drive velocity in meters per second
     *
     * @return Current drive velocity
     */
    private double getDriveVelocity() {
        // Get velocity in ticks per second
        double velocityTicksPerSec = driveMotor.getVelocity();

        // Convert to meters per second
        return velocityTicksPerSec / DriveConstants.TICKS_PER_METER;
    }

    /**
     * Get the drive position in meters
     *
     * @return Current drive position
     */
    private double getDrivePosition() {
        // Get position in ticks
        int positionTicks = driveMotor.getCurrentPosition();

        // Convert to meters
        return positionTicks / DriveConstants.TICKS_PER_METER;
    }

    /**
     * Normalize angle to [-π, π] range
     *
     * @param angle Angle in radians
     * @return Normalized angle
     */
    private double normalizeAngle(double angle) {
        double normalized = angle;
        while (normalized > Math.PI) {
            normalized -= 2 * Math.PI;
        }
        while (normalized < -Math.PI) {
            normalized += 2 * Math.PI;
        }
        return normalized;
    }

    /**
     * Get the module name for telemetry
     *
     * @return Module name (e.g., "FL")
     */
    public String getModuleName() {
        return moduleName;
    }

    /**
     * Get the desired state (for telemetry/debugging)
     *
     * @return Desired module state
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * Get current steering angle for telemetry
     *
     * @return Current steering angle in radians
     */
    public double getCurrentSteeringAngle() {
        return getSteeringAngle();
    }

    /**
     * Stop the module (zero power to drive and steer)
     */
    public void stop() {
        driveMotor.setPower(0.0);
        steerServo.setPower(0.0);
    }
}
