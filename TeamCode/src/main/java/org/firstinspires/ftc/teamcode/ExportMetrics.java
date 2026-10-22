package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;

/**
 * Saves everything in MetricsDatabase to /sdcard/FIRST/data/metrics.csv
 * so it can be downloaded from the Control Hub with adb pull.
 */
@TeleOp(name = "Export Metrics", group = "Metrics")
public class ExportMetrics extends LinearOpMode {

    // /sdcard/FIRST/data/metrics.csv
    private static final File EXPORT_FILE =
            new File(AppUtil.ROBOT_DATA_DIR, "metrics.csv");

    @Override
    public void runOpMode() {

        MetricsDatabase metrics = MetricsDatabase.getInstance();

        telemetry.addData("Metrics stored", metrics.getMetricCount());
        telemetry.addLine("Press START to export to " + EXPORT_FILE.getPath());
        telemetry.update();

        waitForStart();

        try {
            int exported = metrics.exportToCsv(EXPORT_FILE);
            telemetry.addData("Exported", "%d metrics", exported);
            telemetry.addData("File", EXPORT_FILE.getPath());
        } catch (IOException e) {
            telemetry.addData("Export FAILED", e.getMessage());
        }
        telemetry.update();

        // Keep the result on the Driver Hub screen until STOP is pressed
        while (opModeIsActive()) {
            sleep(50);
        }
    }
}
