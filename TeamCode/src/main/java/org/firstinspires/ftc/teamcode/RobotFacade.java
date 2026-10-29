package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotFacade {

    private final HardwareDriveSubsystem drive;
    private final IPerformanceLog performanceLog;

    public RobotFacade(HardwareMap hardwareMap) {
        drive = HardwareFactory.createDriveSubsystem(hardwareMap);
        performanceLog = MetricsDatabase.getInstance();
    }

    public void drive(double forward, double strafe, double turn) {
        drive.drive(forward, strafe, turn);
    }

    public void stopDrive() {
        drive.stop();
    }

    public HardwareDriveSubsystem getDriveSubsystem() {
        return drive;
    }

    public void logMetric(String opMode, String name, double value) {
        performanceLog.logMetric(opMode, name, value);
    }

}
