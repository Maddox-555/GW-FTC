
package org.firstinspires.ftc.teamcode;

/**
 * Interface for recording robot performance metrics.
 * MetricsDatabase implements this interface so other classes
 * can log performance data without depending directly on SQLite.
 */
public interface IPerformanceLog {

    /**
     * Records one performance metric.
     *
     * @param opMode name of the active OpMode
     * @param name   name of the metric
     * @param value  numeric value to record
     */
    void logMetric(String opMode, String name, double value);
}
