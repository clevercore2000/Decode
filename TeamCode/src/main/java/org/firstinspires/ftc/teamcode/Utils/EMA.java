package org.firstinspires.ftc.teamcode.Utils;

/**
 * A simple Exponential Moving Average (EMA) filter.
 */
public class EMA {
    private double alpha;
    private Double lastValue = null;

    /**
     * @param alpha The smoothing factor, between 0.0 and 1.0.
     *              A smaller alpha means more smoothing.
     */
    public EMA(double alpha) {
        this.alpha = alpha;
    }

    /**
     * Updates the filter with the current value.
     * @param currentValue The current sensor value.
     * @return The filtered value.
     */
    public double update(double currentValue) {
        if (lastValue == null) {
            lastValue = currentValue;
            return currentValue;
        }

        double newValue = alpha * currentValue + (1.0 - alpha) * lastValue;
        lastValue = newValue;
        return newValue;
    }

    /**
     * Resets the filter's state.
     */
    public void reset() {
        lastValue = null;
    }
}
