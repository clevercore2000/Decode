package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController
{
    private double kP;
    private double kI;
    private double kD;
    private double setpoint = 0;
    private double processVariable = 0;
    private double output = 0;
    private double error = 0;
    private double previousError = 0;
    private double integral = 0;
    private double maxIntegral = 1.0;
    private double derivative = 0;
    private double deltaTime = 0;
    ElapsedTime timer;

    // Continuous input support for circular quantities (angles)
    private boolean isContinuous = false;
    private double inputMin = 0;
    private double inputMax = 0;
    public PIDController( double pid_Kp, double pid_Ki, double pid_Kd)
    {
        kP = pid_Kp;
        kI = pid_Ki;
        kD = pid_Kd;
        timer = new ElapsedTime();
        timer.startTime();
        timer.reset();
    }

    void setPID( double pid_Kp, double pid_Ki, double pid_Kd )
    {
        kP = pid_Kp;
        kI = pid_Ki;
        kD = pid_Kd;
    }

    double calculate( double targetSetpoint, double currentValue)
    {
        setpoint = targetSetpoint;
        processVariable = currentValue;

        // Read deltaTime BEFORE resetting timer (prevents derivative spikes)
        deltaTime = timer.milliseconds()/1E3;
        timer.reset();

        // Error calculation with optional continuous input wrapping
        error = setpoint - processVariable;
        if (isContinuous) {
            double inputRange = inputMax - inputMin;
            error = ((error + inputRange / 2) % inputRange) - inputRange / 2;
            if (error < -inputRange / 2) {
                error += inputRange;
            }
        }

        // Integral term with windup protection
        integral += error * deltaTime;
        integral = Math.max(-maxIntegral, Math.min(integral, maxIntegral));

        // Derivative term (prevent division by zero)
        if (deltaTime > 0) {
            derivative = (error - previousError) / deltaTime;
        } else {
            derivative = 0;
        }
        previousError = error;

        // Calculate total output
        output = (kP * error) + (kI * integral) + (kD * derivative);

        // Apply limiter
        output = Math.max(-1, Math.min(output, 1));
        return output;
    }

    /**
     * Enable continuous input for circular quantities like angles
     * @param min Minimum input value (e.g., -Math.PI)
     * @param max Maximum input value (e.g., Math.PI)
     */
    public void enableContinuousInput(double min, double max) {
        isContinuous = true;
        inputMin = min;
        inputMax = max;
    }

    /**
     * Disable continuous input
     */
    public void disableContinuousInput() {
        isContinuous = false;
    }
}