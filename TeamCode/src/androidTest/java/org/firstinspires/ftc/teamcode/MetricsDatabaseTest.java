package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotEquals;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import androidx.test.ext.junit.runners.AndroidJUnit4;
import androidx.test.platform.app.InstrumentationRegistry;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;
import org.junit.runner.RunWith;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

/**
 * Checks MetricsDatabase on an emulator. Run with:
 *   ANDROID_SERIAL=emulator-5554 ./gradlew :TeamCode:connectedDebugAndroidTest
 *
 * These tests clear the database, so never run them on the robot.
 */
@RunWith(AndroidJUnit4.class)
public class MetricsDatabaseTest {

    private MetricsDatabase metrics;

    @Before
    public void setUp() {
        metrics = MetricsDatabase.getInstance();
        metrics.clearAllMetrics();
    }

    @After
    public void tearDown() {
        metrics.clearAllMetrics();
    }

    @Test
    public void getInstanceAlwaysReturnsTheSameDatabase() {
        assertSame(MetricsDatabase.getInstance(), MetricsDatabase.getInstance());
    }

    @Test
    public void insertedMetricCanBeReadBack() {
        long before = System.currentTimeMillis();
        long id = metrics.insertMetric("AprilTag Position Test", "tag 5 y", 24.5);

        assertNotEquals(-1, id);

        List<MetricsDatabase.Metric> all = metrics.getAllMetrics();
        assertEquals(1, all.size());

        MetricsDatabase.Metric metric = all.get(0);
        assertEquals(id, metric.id);
        assertEquals("AprilTag Position Test", metric.opMode);
        assertEquals("tag 5 y", metric.name);
        assertEquals(24.5, metric.value, 0.0);
        assertTrue(metric.timestamp >= before);
    }

    @Test
    public void getAllMetricsReturnsOldestFirst() {
        metrics.insertMetric("Test", "first", 1);
        metrics.insertMetric("Test", "second", 2);
        metrics.insertMetric("Test", "third", 3);

        List<MetricsDatabase.Metric> all = metrics.getAllMetrics();

        assertEquals(3, all.size());
        assertEquals("first", all.get(0).name);
        assertEquals("second", all.get(1).name);
        assertEquals("third", all.get(2).name);
    }

    @Test
    public void getMetricsByOpModeOnlyReturnsThatOpMode() {
        metrics.insertMetric("AprilTag Position Test", "tag 5 y", 24.0);
        metrics.insertMetric("Driver's TeleOp", "speed", 1.0);
        metrics.insertMetric("AprilTag Position Test", "tag 5 yaw", 3.0);

        List<MetricsDatabase.Metric> aprilTag = metrics.getMetricsByOpMode("AprilTag Position Test");
        assertEquals(2, aprilTag.size());
        assertEquals("tag 5 y", aprilTag.get(0).name);
        assertEquals("tag 5 yaw", aprilTag.get(1).name);

        // An apostrophe in the name must not break the query
        List<MetricsDatabase.Metric> teleOp = metrics.getMetricsByOpMode("Driver's TeleOp");
        assertEquals(1, teleOp.size());
        assertEquals("speed", teleOp.get(0).name);
    }

    @Test
    public void getMetricCountMatchesInserts() {
        assertEquals(0, metrics.getMetricCount());

        metrics.insertMetric("Test", "a", 1);
        metrics.insertMetric("Test", "b", 2);

        assertEquals(2, metrics.getMetricCount());
    }

    @Test
    public void clearAllMetricsDeletesEverything() {
        metrics.insertMetric("Test", "a", 1);
        metrics.insertMetric("Test", "b", 2);
        metrics.insertMetric("Other", "c", 3);

        assertEquals(3, metrics.clearAllMetrics());
        assertEquals(0, metrics.getMetricCount());
        assertTrue(metrics.getAllMetrics().isEmpty());
    }

    @Test
    public void exportToCsvWritesHeaderAndOneLinePerMetric() throws IOException {
        metrics.insertMetric("AprilTag Position Test", "tag 5 y", 23.75);
        metrics.insertMetric("Test, \"quoted\"", "x", -1.5);

        File file = new File(
                InstrumentationRegistry.getInstrumentation().getTargetContext().getCacheDir(),
                "metrics-test.csv");

        assertEquals(2, metrics.exportToCsv(file));

        List<String> lines = readLines(file);
        assertEquals(3, lines.size());
        assertEquals("id,timestamp_ms,time,op_mode,name,value", lines.get(0));
        assertTrue(lines.get(1), lines.get(1).endsWith(",\"AprilTag Position Test\",\"tag 5 y\",23.75"));
        // Commas and quotes inside a field stay inside one quoted column
        assertTrue(lines.get(2), lines.get(2).endsWith(",\"Test, \"\"quoted\"\"\",\"x\",-1.5"));

        file.delete();
    }

    @Test
    public void exportToCsvCanWriteToTheRobotDataFolder() throws IOException {
        metrics.insertMetric("Test", "a", 1);

        // Same folder the Export Metrics OpMode uses: /sdcard/FIRST/data
        File file = new File(AppUtil.ROBOT_DATA_DIR, "metrics-test.csv");

        assertEquals(1, metrics.exportToCsv(file));
        assertTrue(file.exists());

        file.delete();
    }

    private static List<String> readLines(File file) throws IOException {
        List<String> lines = new ArrayList<>();
        try (BufferedReader in = new BufferedReader(new FileReader(file))) {
            String line;
            while ((line = in.readLine()) != null) {
                lines.add(line);
            }
        }
        return lines;
    }
}
