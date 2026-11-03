package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * RobotFacade implements the Facade design pattern to provide a centralized,
 * simplified interface for controlling robot subsystems and recording performance metrics.
 */
public class RobotFacade {

    private final HardwareDriveSubsystem drive;
    private final IPerformanceLog performanceLog;

    /**
     * Constructs a RobotFacade instance, initializing subsystems via HardwareFactory
     * and connecting to the shared metrics database.
     *
     * @param hardwareMap the FTC hardware map
     */
    public RobotFacade(HardwareMap hardwareMap) {
        drive = HardwareFactory.createDriveSubsystem(hardwareMap);
        performanceLog = MetricsDatabase.getInstance();
    }

    /**
     * Drives the robot drivetrain with the specified forward, strafe, and turn values.
     *
     * @param forward forward power [-1.0, 1.0]
     * @param strafe  strafe power [-1.0, 1.0]
     * @param turn    turn power [-1.0, 1.0]
     */
    public void drive(double forward, double strafe, double turn) {
        drive.drive(forward, strafe, turn);
    }

    /**
     * Stops all drivetrain movement.
     */
    public void stopDrive() {
        drive.stop();
    }

    /**
     * Returns the underlying HardwareDriveSubsystem instance.
     *
     * @return HardwareDriveSubsystem reference
     */
    public HardwareDriveSubsystem getDriveSubsystem() {
        return drive;
    }

    /**
     * Logs a performance metric value for the active OpMode.
     *
     * @param opMode name of the OpMode
     * @param name   metric name
     * @param value  numeric metric value
     */
    public void logMetric(String opMode, String name, double value) {
        performanceLog.logMetric(opMode, name, value);
    }

}
