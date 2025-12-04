package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;
import org.firstinspires.ftc.teamcode.Utils.EMA;

public class AxonEncoder {
    private final AnalogInput encoder;
    private final double voltageOffset;
    private final EMA emaFilter;

    public AxonEncoder(AnalogInput encoder, double voltageOffset) {
        this.encoder = encoder;
        this.voltageOffset = voltageOffset;
        this.emaFilter = new EMA(SteeringConstants.ENCODER_FILTER_ALPHA);
    }

    public double getAngleDegrees() {
        double currentVoltage = encoder.getVoltage();

        // Clamp voltage to valid range (prevents spikes from causing angle errors)
        if (currentVoltage < 0.0) {
            currentVoltage = 0.0;
        }
        if (currentVoltage > SteeringConstants.MAX_VOLTAGE) {
            currentVoltage = SteeringConstants.MAX_VOLTAGE;
        }

        double adjustedVoltage = currentVoltage - voltageOffset;

        if (adjustedVoltage < 0) {
            adjustedVoltage += SteeringConstants.MAX_VOLTAGE;
        }

        double servoAngle = (adjustedVoltage / SteeringConstants.MAX_VOLTAGE) * 720.0;
        double wheelAngle = servoAngle / SteeringConstants.SERVO_TO_WHEEL_RATIO;

        // Apply EMA filter only if alpha is greater than 0
        if (SteeringConstants.ENCODER_FILTER_ALPHA > 0) {
            return emaFilter.update(wheelAngle);
        } else {
            return wheelAngle;
        }
    }

    public double getAngleRadians() {
        return Math.toRadians(getAngleDegrees());
    }

    public double getRawVoltage() {
        return encoder.getVoltage();
    }
}
