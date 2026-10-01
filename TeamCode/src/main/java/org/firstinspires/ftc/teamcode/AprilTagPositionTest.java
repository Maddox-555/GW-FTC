package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "AprilTag Position Test", group = "Vision")
public class AprilTagPositionTest extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // Desired Hive alignment
    private static final double DESIRED_DISTANCE = 24.0;  // inches
    private static final double DESIRED_X = 0.0;          // inches
    private static final double DESIRED_YAW = 0.0;        // degrees

    // Acceptable alignment tolerances
    private static final double DISTANCE_TOLERANCE = 2.0; // inches
    private static final double X_TOLERANCE = 2.0;        // inches
    private static final double YAW_TOLERANCE = 5.0;      // degrees

    @Override
    public void runOpMode() {

        /*
         * Create the AprilTag processor.
         */
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        /*
         * Create the VisionPortal and connect
         * the Logitech C920X webcam.
         */
        visionPortal = new VisionPortal.Builder()
                .setCamera(
                        hardwareMap.get(
                                WebcamName.class,
                                "C920X"
                        )
                )
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("AprilTag processor initialized");
        telemetry.addData("Camera", "C920X");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData(
                    "Camera State",
                    visionPortal.getCameraState()
            );

            /*
             * Get the current AprilTag detections.
             */
            List<AprilTagDetection> detections =
                    aprilTag.getDetections();

            telemetry.addData(
                    "Detections",
                    detections.size()
            );

            /*
             * Process every detection.
             */
            for (AprilTagDetection detection : detections) {

                /*
                 * -------------------------------------------------
                 * SINGLE APRILTAG
                 * -------------------------------------------------
                 */
                if (detection instanceof AprilTagSingleDetection) {

                    AprilTagSingleDetection singleDet =
                            (AprilTagSingleDetection) detection;

                    telemetry.addLine("Single AprilTag Detected");

                    telemetry.addData(
                            "Tag ID",
                            singleDet.id
                    );

                    /*
                     * A single tag may or may not have metadata.
                     * Pose requires metadata containing tag size.
                     */
                    if (singleDet.metadata != null) {

                        telemetry.addData(
                                "Tag Name",
                                singleDet.metadata.name
                        );

                    } else {

                        telemetry.addLine(
                                "Tag Metadata Unavailable"
                        );
                    }
                }

                /*
                 * -------------------------------------------------
                 * APRILTAG CLUSTER
                 * -------------------------------------------------
                 */
                else if (detection instanceof AprilTagClusterDetection) {

                    AprilTagClusterDetection clusterDet =
                            (AprilTagClusterDetection) detection;

                    telemetry.addLine(
                            "AprilTag Cluster Detected"
                    );

                    /*
                     * Clusters do not have an ID.
                     * They are identified by their metadata name.
                     */
                    telemetry.addData(
                            "Cluster",
                            clusterDet.metadata.name
                    );

                    /*
                     * percentClusterFound is already an integer
                     * percentage in FTC SDK 12.0.
                     */
                    telemetry.addData(
                            "Cluster Visible",
                            "%d%%",
                            clusterDet.percentClusterFound
                    );
                }

                /*
                 * -------------------------------------------------
                 * POSE INFORMATION
                 * -------------------------------------------------
                 *
                 * ftcPose is declared on AprilTagDetection itself,
                 * so it works for both single tags and clusters.
                 */
                if (detection.ftcPose != null) {

                    double x = detection.ftcPose.x;
                    double y = detection.ftcPose.y;
                    double z = detection.ftcPose.z;

                    double yaw = detection.ftcPose.yaw;
                    double pitch = detection.ftcPose.pitch;
                    double roll = detection.ftcPose.roll;

                    double range = detection.ftcPose.range;
                    double bearing = detection.ftcPose.bearing;
                    double elevation = detection.ftcPose.elevation;

                    telemetry.addData(
                            "X",
                            "%.2f in",
                            x
                    );

                    telemetry.addData(
                            "Y",
                            "%.2f in",
                            y
                    );

                    telemetry.addData(
                            "Z",
                            "%.2f in",
                            z
                    );

                    telemetry.addData(
                            "Yaw",
                            "%.2f deg",
                            yaw
                    );

                    telemetry.addData(
                            "Pitch",
                            "%.2f deg",
                            pitch
                    );

                    telemetry.addData(
                            "Roll",
                            "%.2f deg",
                            roll
                    );

                    telemetry.addData(
                            "Range",
                            "%.2f in",
                            range
                    );

                    telemetry.addData(
                            "Bearing",
                            "%.2f deg",
                            bearing
                    );

                    telemetry.addData(
                            "Elevation",
                            "%.2f deg",
                            elevation
                    );

                    /*
                     * -------------------------------------------------
                     * TARGET ALIGNMENT
                     * -------------------------------------------------
                     *
                     * Y is the forward distance in the FTC
                     * reference frame.
                     */
                    double distanceError =
                            y - DESIRED_DISTANCE;

                    double xError =
                            x - DESIRED_X;

                    double yawError =
                            yaw - DESIRED_YAW;

                    telemetry.addData(
                            "Distance Error",
                            "%.2f in",
                            distanceError
                    );

                    telemetry.addData(
                            "X Error",
                            "%.2f in",
                            xError
                    );

                    telemetry.addData(
                            "Yaw Error",
                            "%.2f deg",
                            yawError
                    );

                    /*
                     * Determine whether the camera is
                     * within the desired alignment window.
                     */
                    boolean distanceOK =
                            Math.abs(distanceError)
                                    <= DISTANCE_TOLERANCE;

                    boolean xOK =
                            Math.abs(xError)
                                    <= X_TOLERANCE;

                    boolean yawOK =
                            Math.abs(yawError)
                                    <= YAW_TOLERANCE;

                    if (distanceOK && xOK && yawOK) {

                        telemetry.addLine(
                                "TARGET POSITION OK"
                        );

                    } else {

                        telemetry.addLine(
                                "ADJUST POSITION"
                        );
                    }

                } else {

                    telemetry.addLine(
                            "Pose Unavailable"
                    );
                }
            }

            telemetry.update();
        }

        /*
         * Close the VisionPortal when the OpMode ends.
         */
        visionPortal.close();
    }
}

