package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class SlewRateLimiter {
    private final double m_positiveRateLimit;
    private final double m_negativeRateLimit;
    private final ElapsedTime timer = new ElapsedTime();
    private double m_prevVal;
    private double m_prevTime;

    /**
     * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *     second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *     second. This is expected to be negative.
     * @param initialValue The initial value of the input.
     */
    public SlewRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue) {
        m_positiveRateLimit = positiveRateLimit;
        m_negativeRateLimit = negativeRateLimit;
        m_prevVal = initialValue;
        timer.startTime();
        m_prevTime = timer.seconds();
    }

    /**
     * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public SlewRateLimiter(double rateLimit) {
        this(rateLimit, -rateLimit, 0);
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = timer.seconds();
        double elapsedTime = currentTime - m_prevTime;
        m_prevVal +=
                Range.clip(
                        input - m_prevVal,
                        m_negativeRateLimit * elapsedTime,
                        m_positiveRateLimit * elapsedTime);
        m_prevTime = currentTime;
        return m_prevVal;
    }

    /**
     * Returns the value last calculated by the SlewRateLimiter.
     *
     * @return The last value.
     */
    public double lastValue() {
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_prevVal = value;
        timer.reset();
        timer.startTime();
        m_prevTime = timer.seconds();
    }
}
