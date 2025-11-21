package org.firstinspires.ftc.teamcode.Subsystems.Utils;

public class EMAFilter {
    private double previousValue = 0;
    private double currentValue = 0;

    public double filter(double inputValue, double alpha) {
        currentValue = alpha * inputValue + (1 - alpha) * previousValue;
        previousValue = currentValue;
        return currentValue;
    }

    public void reset() {
        previousValue = 0;
        currentValue = 0;
    }
}