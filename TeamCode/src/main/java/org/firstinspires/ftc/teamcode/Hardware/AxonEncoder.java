package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Utils.EMA;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

/**
 * Axon encoder for swerve module steering.
 * Encoder outputs 0-3.3V for full 360° wheel rotation.
 *
 * Two angle reading methods:
 * - getAngleDegrees(): Returns [0, 360) - legacy method
 * - getRawAngleDegrees180(): Returns [-180, 180] - for continuous tracking (Cristian's fix)
 *
 * CRITICAL: Calibration offset is applied so 0° = wheel pointing forward.
 */
public class AxonEncoder {
    private final AnalogInput encoder;
    private final double voltageOffset;  // Calibration voltage when wheel points forward (0°)
    private final double angleOffsetDegrees;  // Calibration offset in wheel angle degrees (legacy)
    private final EMA emaFilter;

    /**
     * @param encoder The analog input for the encoder
     * @param voltageOffset The voltage reading when wheel points forward (0°)
     */
    public AxonEncoder(AnalogInput encoder, double voltageOffset) {
        this.encoder = encoder;
        this.voltageOffset = voltageOffset;
        // Convert voltage offset to wheel angle offset (for legacy getAngleDegrees())
        this.angleOffsetDegrees = voltageToWheelAngle(voltageOffset);
        this.emaFilter = new EMA(SteeringConstants.ENCODER_FILTER_ALPHA);
    }

    /**
     * Converts raw voltage to wheel angle WITHOUT offset (for offset calculation).
     * Encoder outputs 0-3.3V for full 360° wheel rotation.
     */
    private double voltageToWheelAngle(double voltage) {
        // Clamp voltage to valid range
        voltage = Math.max(0.0, Math.min(voltage, SteeringConstants.MAX_VOLTAGE));

        // Voltage 0-3.3V → Wheel angle 0-360°
        return (voltage / SteeringConstants.MAX_VOLTAGE) * 360.0;
    }

    /**
     * Normalizes angle to [0, 360) range.
     * Handles wrapping: 361° → 1°, -10° → 350°
     */
    private double normalizeAngle0To360(double angleDegrees) {
        // Use modulo to bring to (-360, 360) range first
        angleDegrees = angleDegrees % 360.0;

        // Handle negative angles: -10° → 350°
        if (angleDegrees < 0) {
            angleDegrees += 360.0;
        }

        return angleDegrees;
    }

    public double getAngleDegrees() {
        double currentVoltage = encoder.getVoltage();

        // Convert voltage to raw wheel angle (before offset)
        double rawWheelAngle = voltageToWheelAngle(currentVoltage);

        // Apply calibration offset IN THE ANGLE DOMAIN (not voltage domain!)
        // This ensures proper wrapping with the gear ratio
        double calibratedAngle = rawWheelAngle - angleOffsetDegrees;

        // Normalize to [0, 360) to handle wrapping at 0°/360° boundary
        // e.g., if calibratedAngle = -10°, it becomes 350°
        //       if calibratedAngle = 370°, it becomes 10°
        double normalizedAngle = normalizeAngle0To360(calibratedAngle);

        // Apply EMA filter only if alpha is greater than 0
        if (SteeringConstants.ENCODER_FILTER_ALPHA > 0) {
            return emaFilter.update(normalizedAngle);
        } else {
            return normalizedAngle;
        }
    }

    public double getAngleRadians() {
        return Math.toRadians(getAngleDegrees());
    }

    public double getRawVoltage() {
        return encoder.getVoltage();
    }

    /**
     * Returns the raw servo angle (0-720°) for debugging.
     * Useful for verifying gear ratio and encoder functionality.
     */
    public double getRawServoAngle() {
        double voltage = Math.max(0.0, Math.min(encoder.getVoltage(), SteeringConstants.MAX_VOLTAGE));
        return (voltage / SteeringConstants.MAX_VOLTAGE) * 720.0;
    }

    /**
     * Gets calibrated wheel angle in [-180, 180] range.
     * Used for continuous tracking with delta unwrapping in SwerveModule.
     *
     * Encoder outputs 0-3.3V for full 360° rotation.
     * Formula: (voltage / 3.3) * 360 - 180 = [-180, +180]
     * Offset is applied so 0° = wheel pointing forward.
     *
     * @return Calibrated wheel angle in [-180, 180] degrees
     */
    public double getRawAngleDegrees180() {
        double voltage = encoder.getVoltage();
        // Raw angle: 0-3.3V → -180 to +180 degrees
        double rawAngle = ((voltage / 3.3) * 360.0) - 180.0;
        // Offset angle (what raw reads when wheel is forward)
        double offsetAngle = ((voltageOffset / 3.3) * 360.0) - 180.0;
        // Calibrated angle: 0° when wheel points forward
        double calibrated = rawAngle - offsetAngle;
        // Wrap to [-180, 180]
        return MathUtils.wrap180(calibrated);
    }
}
