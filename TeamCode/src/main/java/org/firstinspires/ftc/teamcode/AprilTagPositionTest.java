package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/*
 * Shows whether the robot is in position to score, using the same camera, target
 * and "ready" check as the Auto, and records what the camera sees to the metrics
 * database. Also shows the numbers needed to measure the camera tilt and the
 * HIVE cluster height (see RobotConstants).
 */
@TeleOp(name = "Hive Scoring Decision Test", group = "Vision")
public class AprilTagPositionTest extends LinearOpMode {

    private AprilTagVisionSystem visionSystem;

    private String alliance = "RED";

    // Metrics recording (saved to MetricsDatabase)
    private static final String OP_MODE_NAME = "Hive Scoring Decision Test";
    private static final double RECORD_INTERVAL_MS = 250; // 4 samples per second

    private MetricsDatabase metrics;
    private int metricsRecorded = 0;

    @Override
    public void runOpMode() {

        visionSystem = new AprilTagVisionSystem(hardwareMap);

        metrics = MetricsDatabase.getInstance();
        ElapsedTime recordTimer = new ElapsedTime();

        telemetry.addLine("Hive Scoring Decision Test");
        telemetry.addLine("Connect DualShock 4 to Driver Hub as gamepad 2");
        telemetry.addLine("Cross = Score request");
        telemetry.addLine("Circle = Cancel");
        telemetry.addLine("D-pad LEFT = Red alliance");
        telemetry.addLine("D-pad RIGHT = Blue alliance");
        telemetry.addData("Aiming with", ShotGeometry.usesFloorGeometry()
                ? "floor distance" : "camera numbers");
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
                telemetry.addData("Alliance", alliance);

                List<AprilTagDetection> detections =
                        visionSystem.getDetections();

                // Save a sample every RECORD_INTERVAL_MS so database
                // writes don't slow the loop down
                if (recordTimer.milliseconds() >= RECORD_INTERVAL_MS) {
                    recordTimer.reset();
                    recordDetections(detections);
                }

                // Same list as the one recorded above, so both come from one camera frame
                AprilTagDetection target =
                        AprilTagVisionSystem.findScoringTarget(detections, alliance);

                boolean targetReady = false;
                String decision;

                if (target == null) {
                    decision = "SEARCHING FOR " + alliance + " CELL";
                } else {
                    ShotGeometry shot = new ShotGeometry(target.ftcPose);
                    targetReady = shot.ready();

                    telemetry.addData("Target", AprilTagVisionSystem.targetName(target));
                    telemetry.addData("Distance", "%.1f in (want %.1f)",
                            shot.distance, shot.desiredDistance);
                    telemetry.addData("Side", "%.1f in", shot.side);
                    telemetry.addData("Heading", "%.1f deg", shot.headingDeg);
                    telemetry.addData("Upright", shot.upright);
                    telemetry.addData("Correct distance", shot.correctDistance());
                    telemetry.addData("Centered", shot.centered());
                    telemetry.addData("Aimed", shot.aimed());

                    // Numbers for measuring (see RobotConstants)
                    telemetry.addLine("--- measuring ---");
                    telemetry.addData("x / y / z", "%.1f / %.1f / %.1f in",
                            target.ftcPose.x, target.ftcPose.y, target.ftcPose.z);
                    telemetry.addData("3D distance", "%.1f in", shot.range3d);
                    if (!Double.isNaN(shot.heightFromTilt)) {
                        telemetry.addData("Cluster height above camera", "%.1f in",
                                shot.heightFromTilt);
                    }
                    if (!Double.isNaN(shot.floorDistance)) {
                        telemetry.addData("Floor distance", "%.1f in", shot.floorDistance);
                        telemetry.addData("Camera tilt seen", "%.1f deg", shot.measuredTiltDeg);
                    }
                    if (shot.tooCloseToMeasure) {
                        telemetry.addLine("TOO CLOSE TO MEASURE - move back");
                    }
                    if (shot.cameraMoved()) {
                        telemetry.addLine("CAMERA MOVED? check the camera mount");
                    }

                    decision = targetReady
                            ? "TARGET READY"
                            : "ADJUST POSITION";
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

                telemetry.addData("Camera", visionSystem.getCameraState());
                telemetry.addData("Decision", decision);
                telemetry.addData("Detections", detections.size());
                telemetry.addData("Metrics Recorded", metricsRecorded);
                telemetry.addLine("Cross = request | Circle = cancel");
                telemetry.update();
            }
        } finally {
            visionSystem.close();
        }
    }

    /*
     * Saves the detection count, plus x / y / z / bearing for every
     * tag or cluster that has a pose. This records everything the
     * camera sees, not just the alliance cell.
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
                source = AprilTagVisionSystem.targetName(detection);
            }

            recordMetric(source + " x", detection.ftcPose.x);
            recordMetric(source + " y", detection.ftcPose.y);
            recordMetric(source + " z", detection.ftcPose.z);
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
