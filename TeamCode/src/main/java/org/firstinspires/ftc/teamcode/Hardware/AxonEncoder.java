package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.Constants.SteeringConstants;

public class AxonEncoder {
    private final AnalogInput encoder;
    private final double voltageOffset;

    public AxonEncoder(AnalogInput encoder, double voltageOffset) {
        this.encoder = encoder;
        this.voltageOffset = voltageOffset;
    }

    public double getAngleDegrees() {
        double currentVoltage = encoder.getVoltage();
        double adjustedVoltage = currentVoltage - voltageOffset;

        if (adjustedVoltage < 0) {
            adjustedVoltage += SteeringConstants.MAX_VOLTAGE;
        }

        double servoAngle = (adjustedVoltage / SteeringConstants.MAX_VOLTAGE) * 720.0;
        double wheelAngle = servoAngle / SteeringConstants.SERVO_TO_WHEEL_RATIO;

        return wheelAngle;
    }

    public double getAngleRadians() {
        return Math.toRadians(getAngleDegrees());
    }

    public double getRawVoltage() {
        return encoder.getVoltage();
    }
}
