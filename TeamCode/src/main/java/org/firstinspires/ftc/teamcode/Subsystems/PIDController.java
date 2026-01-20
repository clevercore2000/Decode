package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double kP, kI, kD;
    private double setpoint = 0;
    private double processVariable = 0;
    private double output = 0;
    private double error = 0;
    private double previousError = 0;
    private double integral = 0;
    private double maxIntegral = 1.0;
    private double derivative = 0;
    private double deltaTime = 0;
    private ElapsedTime timer;

    private boolean isContinuous = false;
    private double inputMin = 0;
    private double inputMax = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        timer = new ElapsedTime();
        timer.startTime();
        timer.reset();
    }

    public void setPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double targetSetpoint, double currentValue) {
        setpoint = targetSetpoint;
        processVariable = currentValue;

        deltaTime = timer.milliseconds() / 1E3;
        timer.reset();

        error = setpoint - processVariable;
        if (isContinuous) {
            double inputRange = inputMax - inputMin;
            error = ((error + inputRange / 2) % inputRange) - inputRange / 2;
            if (error < -inputRange / 2) error += inputRange;
        }

        integral += error * deltaTime;
        integral = Math.max(-maxIntegral, Math.min(integral, maxIntegral));

        if (deltaTime > 0) {
            derivative = (error - previousError) / deltaTime;
        } else {
            derivative = 0;
        }
        previousError = error;

        output = (kP * error) + (kI * integral) + (kD * derivative);
        output = Math.max(-1, Math.min(output, 1));
        return output;
    }

    public void enableContinuousInput(double min, double max) {
        isContinuous = true;
        inputMin = min;
        inputMax = max;
    }

    public void disableContinuousInput() {
        isContinuous = false;
    }
}
