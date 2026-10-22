package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "Hive Scoring Decision Test", group = "Vision")
public class AprilTagPositionTest extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private String alliance = "RED";

    private static final double DESIRED_DISTANCE = 24.0;
    private static final double DISTANCE_TOLERANCE = 2.0;
    private static final double X_TOLERANCE = 2.0;
    private static final double BEARING_TOLERANCE = 5.0;

    // Metrics recording (saved to MetricsDatabase)
    private static final String OP_MODE_NAME = "Hive Scoring Decision Test";
    private static final double RECORD_INTERVAL_MS = 250; // 4 samples per second

    private MetricsDatabase metrics;
    private int metricsRecorded = 0;

    @Override
    public void runOpMode() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "C920X"))
                .addProcessor(aprilTag)
                .build();

        metrics = MetricsDatabase.getInstance();
        ElapsedTime recordTimer = new ElapsedTime();

        telemetry.addLine("Hive Scoring Decision Test");
        telemetry.addLine("Connect DualShock 4 to Driver Hub");
        telemetry.addLine("Cross = Score request");
        telemetry.addLine("Circle = Cancel");
        telemetry.addLine("D-pad LEFT = Red alliance");
        telemetry.addLine("D-pad RIGHT = Blue alliance");
        telemetry.update();

        waitForStart();

        boolean previousA = false;

        try {
            while (opModeIsActive()) {

                // DualShock 4 controls
                if (gamepad2.dpad_left) {
                    alliance = "RED";
                }

                if (gamepad2.dpad_right) {
                    alliance = "BLUE";
                }

                boolean scorePressed = gamepad2.a;
                boolean cancelPressed = gamepad2.b;

                // Display controller input so you can verify the mapping
                telemetry.addData("Cross / A", gamepad2.a);
                telemetry.addData("Circle / B", gamepad2.b);
                telemetry.addData("Square / X", gamepad2.x);
                telemetry.addData("Triangle / Y", gamepad2.y);
                telemetry.addData("Alliance", alliance);

                boolean targetFound = false;
                boolean targetReady = false;
                String targetName = "No scoring cluster";
                String decision = "SEARCHING";

                List<AprilTagDetection> detections =
                        aprilTag.getDetections();

                // Save a sample every RECORD_INTERVAL_MS so database
                // writes don't slow the loop down
                if (recordTimer.milliseconds() >= RECORD_INTERVAL_MS) {
                    recordTimer.reset();
                    recordDetections(detections);
                }

                for (AprilTagDetection detection : detections) {

                    if (!(detection instanceof AprilTagClusterDetection)) {
                        continue;
                    }

                    AprilTagClusterDetection cluster =
                            (AprilTagClusterDetection) detection;

                    if (cluster.metadata == null
                            || cluster.metadata.name == null) {
                        continue;
                    }

                    String name = cluster.metadata.name;

                    if (!name.equalsIgnoreCase(alliance + " SCORING")) {
                        continue;
                    }

                    targetFound = true;
                    targetName = name;

                    if (detection.ftcPose == null) {
                        decision = "TARGET POSE UNAVAILABLE";
                        break;
                    }

                    double x = detection.ftcPose.x;
                    double y = detection.ftcPose.y;
                    double bearing = detection.ftcPose.bearing;
                    double roll = detection.ftcPose.roll;

                    boolean upright = Math.abs(roll) < 90.0;
                    boolean correctDistance =
                            Math.abs(y - DESIRED_DISTANCE)
                                    <= DISTANCE_TOLERANCE;
                    boolean centered =
                            Math.abs(x) <= X_TOLERANCE;
                    boolean aimed =
                            Math.abs(bearing) <= BEARING_TOLERANCE;

                    targetReady = upright
                            && correctDistance
                            && centered
                            && aimed;

                    telemetry.addData("Target X", "%.1f in", x);
                    telemetry.addData("Target Y", "%.1f in", y);
                    telemetry.addData("Bearing", "%.1f deg", bearing);
                    telemetry.addData("Roll", "%.1f deg", roll);
                    telemetry.addData("Upright", upright);
                    telemetry.addData("Correct distance", correctDistance);
                    telemetry.addData("Centered", centered);
                    telemetry.addData("Aimed", aimed);

                    decision = targetReady
                            ? "TARGET READY"
                            : "ADJUST POSITION";

                    break;
                }

                if (!targetFound) {
                    decision = "SEARCHING FOR " + alliance + " SCORING";
                }

                // Circle cancels; Cross requests a simulated score.
                if (cancelPressed) {
                    decision = "SCORE REQUEST CANCELLED";
                } else if (scorePressed && !previousA) {
                    if (targetReady) {
                        decision = "SCORE REQUEST ACCEPTED - SIMULATION ONLY";
                    } else {
                        decision = "SCORE REQUEST BLOCKED - NOT READY";
                    }
                }

                previousA = scorePressed;

                telemetry.addData("Camera", visionPortal.getCameraState());
                telemetry.addData("Target", targetName);
                telemetry.addData("Decision", decision);
                telemetry.addData("Detections", detections.size());
                telemetry.addData("Metrics Recorded", metricsRecorded);
                telemetry.addLine("Cross = request | Circle = cancel");
                telemetry.update();
            }
        } finally {
            visionPortal.close();
        }
    }

    /*
     * Saves the detection count, plus x / y / bearing for every
     * tag or cluster that has a pose. This records everything the
     * camera sees, not just the alliance scoring cluster.
     */
    private void recordDetections(List<AprilTagDetection> detections) {
        recordMetric("detections", detections.size());

        for (AprilTagDetection detection : detections) {

            if (detection.ftcPose == null) {
                continue;
            }

            // Label for this detection's rows, e.g. "tag 12" or "RED SCORING"
            String source = "unknown";

            if (detection instanceof AprilTagSingleDetection) {
                source = "tag " + ((AprilTagSingleDetection) detection).id;
            } else if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;
                if (cluster.metadata != null && cluster.metadata.name != null) {
                    source = cluster.metadata.name;
                }
            }

            recordMetric(source + " x", detection.ftcPose.x);
            recordMetric(source + " y", detection.ftcPose.y);
            recordMetric(source + " bearing", detection.ftcPose.bearing);
        }
    }

    /*
     * Saves one value to the metrics database and counts it
     * for the "Metrics Recorded" telemetry line.
     */
    private void recordMetric(String name, double value) {
        if (metrics.insertMetric(OP_MODE_NAME, name, value) != -1) {
            metricsRecorded++;
        }
    }
}
