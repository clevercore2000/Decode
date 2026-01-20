package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Utils.EMA;
import org.firstinspires.ftc.teamcode.Utils.MathUtils;

/**
 * Axon encoder for swerve steering. Outputs 0-3.3V for 360° wheel rotation.
 * Calibration offset makes 0° = wheel forward.
 */
public class AxonEncoder {
    private final AnalogInput encoder;
    private final double voltageOffset;
    private final double angleOffsetDegrees;
    private final EMA emaFilter;

    public AxonEncoder(AnalogInput encoder, double voltageOffset) {
        this.encoder = encoder;
        this.voltageOffset = voltageOffset;
        this.angleOffsetDegrees = voltageToWheelAngle(voltageOffset);
        this.emaFilter = new EMA(SteeringConstants.ENCODER_FILTER_ALPHA);
    }

    private double voltageToWheelAngle(double voltage) {
        voltage = Math.max(0.0, Math.min(voltage, SteeringConstants.MAX_VOLTAGE));
        return (voltage / SteeringConstants.MAX_VOLTAGE) * 360.0;
    }

    private double normalizeAngle0To360(double angleDegrees) {
        angleDegrees = angleDegrees % 360.0;
        if (angleDegrees < 0) angleDegrees += 360.0;
        return angleDegrees;
    }

    public double getAngleDegrees() {
        double currentVoltage = encoder.getVoltage();
        double rawWheelAngle = voltageToWheelAngle(currentVoltage);
        double calibratedAngle = rawWheelAngle - angleOffsetDegrees;
        double normalizedAngle = normalizeAngle0To360(calibratedAngle);

        if (SteeringConstants.ENCODER_FILTER_ALPHA > 0) {
            return emaFilter.update(normalizedAngle);
        }
        return normalizedAngle;
    }

    public double getAngleRadians() {
        return Math.toRadians(getAngleDegrees());
    }

    public double getRawVoltage() {
        return encoder.getVoltage();
    }

    public double getRawServoAngle() {
        double voltage = Math.max(0.0, Math.min(encoder.getVoltage(), SteeringConstants.MAX_VOLTAGE));
        return (voltage / SteeringConstants.MAX_VOLTAGE) * 720.0;
    }

    /** Returns calibrated wheel angle in [-180, 180] for continuous tracking */
    public double getRawAngleDegrees180() {
        double voltage = encoder.getVoltage();
        double rawAngle = ((voltage / 3.3) * 360.0) - 180.0;
        double offsetAngle = ((voltageOffset / 3.3) * 360.0) - 180.0;
        double calibrated = rawAngle - offsetAngle;
        return MathUtils.wrap180(calibrated);
    }
}
