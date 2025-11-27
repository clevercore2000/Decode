package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;

/**
 * Individual Swerve Module Controller
 *
 * Controls one swerve module (drive motor + steering servo + analog encoder).
 * Uses simple PID steering with 180° flip optimization.
 *
 * Key Features:
 * - PID steering control (P=0.6, D=0.1) with static friction compensation
 * - State optimization: never rotates >90° (reverses drive instead)
 * - Encoder power maintenance (keeps servo PWM active for voltage reading)
 * - EMA filtering to reduce encoder noise
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

    // Encoder filtering
    private double filteredAngle = 0.0;

    /**
     * Create a new swerve module
     *
     * @param driveMotor      Drive motor (DcMotorEx for velocity control)
     * @param steerServo      Steering servo (CRServo mode)
     * @param steerEncoder    Analog encoder (0-3.3V = 0-360°)
     * @param angleOffset     Calibration offset in radians
     * @param driveInverted   Reverse drive motor direction
     * @param steerInverted   Reverse steering servo direction
     * @param moduleName      Name for telemetry (e.g., "FL")
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

        // Initialize PID controller
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
     * Set the desired state for this module
     *
     * This method handles:
     * 1. State optimization (180° flip if needed)
     * 2. PID steering control
     * 3. Static friction compensation
     * 4. Minimum power enforcement (keeps encoder active)
     * 5. Drive motor power application
     *
     * @param targetSpeed Target drive speed [-1.0, 1.0]
     * @param targetAngle Target steering angle in radians
     */
    public void setDesiredState(double targetSpeed, double targetAngle) {
        // Get current steering angle
        double currentAngle = getSteeringAngle();

        // Calculate shortest path angle error
        double angleError = normalizeAngle(targetAngle - currentAngle);

        // STATE OPTIMIZATION: If rotation > 90°, flip 180° and reverse drive
        boolean flipDrive = false;
        if (Math.abs(angleError) > Math.PI / 2) {
            targetAngle = normalizeAngle(targetAngle - Math.PI);
            angleError = normalizeAngle(targetAngle - currentAngle);
            flipDrive = true;
        }

        // Simple proportional control based on normalized error
        // PID libraries don't handle angle wrapping well, so we calculate error manually
        double steerPower = SteeringConstants.STEER_P * angleError;

        steerPower = Range.clip(
            steerPower,
            -SteeringConstants.MAX_SERVO_POWER,
            SteeringConstants.MAX_SERVO_POWER
        );

        // Safety: handle NaN (can occur during rapid changes)
        if (Double.isNaN(steerPower)) {
            steerPower = 0;
        }

        // Static friction compensation when error is significant
        if (Math.abs(angleError) > SteeringConstants.ERROR_THRESHOLD_FOR_STATIC) {
            steerPower += SteeringConstants.K_STATIC * Math.signum(steerPower);
        }

        // CRITICAL: Keep servo powered to maintain encoder readings!
        // Axon encoders require PWM signals to output voltage
        if (Math.abs(steerPower) < SteeringConstants.MIN_SERVO_POWER) {
            steerPower = SteeringConstants.MIN_SERVO_POWER * Math.signum(angleError);
        }

        // Apply steering power (with inversion if needed)
        steerServo.setPower(steerInverted ? -steerPower : steerPower);

        // Apply drive power (with flip if needed)
        double drivePower = flipDrive ? -targetSpeed : targetSpeed;
        drivePower = Range.clip(drivePower, -1.0, 1.0);
        driveMotor.setPower(drivePower);
    }

    /**
     * Get the current steering angle in radians
     *
     * Applies:
     * - Voltage to angle conversion (0-3.3V = 0-2π)
     * - EMA filtering (reduces noise while maintaining responsiveness)
     * - Calibration offset
     * - Angle normalization to [-π, π]
     *
     * @return Current steering angle in radians, range [-π, π]
     */
    private double getSteeringAngle() {
        // Read analog voltage (0-3.3V = 0-360° wheel angle)
        double rawVoltage = steerEncoder.getVoltage();

        // Convert voltage to radians (0-3.3V = 0-2π)
        double rawAngle = (rawVoltage / SteeringConstants.ANALOG_VOLTAGE_MAX) * 2 * Math.PI;

        // EMA filter (95% new, 5% old) - minimal lag
        filteredAngle = SteeringConstants.ENCODER_FILTER_ALPHA * rawAngle
                      + (1 - SteeringConstants.ENCODER_FILTER_ALPHA) * filteredAngle;

        // Apply calibration offset and normalize
        double angle = filteredAngle - angleOffset;
        return normalizeAngle(angle);
    }

    /**
     * Get raw encoder voltage for debugging
     *
     * @return Encoder voltage in range [0, 3.3]
     */
    public double getRawEncoderVoltage() {
        return steerEncoder.getVoltage();
    }

    /**
     * Get current steering angle (public for telemetry)
     *
     * @return Current angle in radians, range [-π, π]
     */
    public double getCurrentSteeringAngle() {
        return getSteeringAngle();
    }

    /**
     * Get current drive motor power
     *
     * @return Motor power in range [-1.0, 1.0]
     */
    public double getDrivePower() {
        return driveMotor.getPower();
    }

    /**
     * Get module name for telemetry
     *
     * @return Module name (e.g., "FL")
     */
    public String getModuleName() {
        return moduleName;
    }

    /**
     * Stop the module
     *
     * CRITICAL: Sets servo to MIN_SERVO_POWER (not 0.0) to keep encoder active!
     */
    public void stop() {
        driveMotor.setPower(0.0);
        steerServo.setPower(SteeringConstants.MIN_SERVO_POWER);  // Keep encoder alive!
    }

    /**
     * Normalize angle to [-π, π] range
     *
     * @param angle Input angle in radians
     * @return Normalized angle in range [-π, π]
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
}
