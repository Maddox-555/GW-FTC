package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * AprilTagVisionSystem encapsulates VisionPortal and AprilTagProcessor
 * to provide robust AprilTag detection and alliance scoring target identification.
 */
public class AprilTagVisionSystem implements ITagPoseData {

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

    /**
     * Initializes the webcam and builds the VisionPortal with AprilTag processing.
     *
     * @param hardwareMap the FTC hardware map
     */
    public AprilTagVisionSystem(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(
                        hardwareMap.get(WebcamName.class, RobotConstants.WEBCAM_NAME))
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Returns the alliance HIVE cell to shoot at: either the SCORING or the AUDIENCE cluster.
     * After each TIP the other cell faces up. The tags of a downward-facing cell point up,
     * away from the camera, so the cluster the camera can see should be the upward-facing
     * cell. If both are seen, the one with more tags found wins.
     *
     * @param alliance alliance string ("RED" or "BLUE")
     * @return the alliance cluster detection with a pose, or null if none is seen
     */
    @Override
    public AprilTagDetection findScoringTarget(String alliance) {
        return findScoringTarget(getDetections(), alliance);
    }

    /**
     * Same as findScoringTarget(alliance), but searches a list of detections that was
     * already fetched, so the target comes from the same camera frame as that list.
     */
    public static AprilTagDetection findScoringTarget(
            List<AprilTagDetection> detections, String alliance) {
        AprilTagClusterDetection best = null;

        for (AprilTagDetection detection : detections) {

            if (!(detection instanceof AprilTagClusterDetection)
                    || detection.ftcPose == null) {
                continue;
            }

            AprilTagClusterDetection cluster =
                    (AprilTagClusterDetection) detection;

            if (cluster.metadata == null
                    || cluster.metadata.name == null) {
                continue;
            }

            String name = cluster.metadata.name;
            boolean allianceCell = name.equalsIgnoreCase(alliance + " SCORING")
                    || name.equalsIgnoreCase(alliance + " AUDIENCE");

            if (allianceCell
                    && (best == null || cluster.percentClusterFound > best.percentClusterFound)) {
                best = cluster;
            }
        }

        return best;
    }

    /**
     * Returns the cluster's name ("RED SCORING", etc.), or "none" if the detection isn't a named cluster.
     */
    public static String targetName(AprilTagDetection detection) {
        if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if (cluster.metadata != null && cluster.metadata.name != null) {
                return cluster.metadata.name;
            }
        }
        return "none";
    }

    /**
     * Returns the current operational state of the VisionPortal camera.
     *
     * @return camera state string
     */
    public String getCameraState() {
        return visionPortal.getCameraState().toString();
    }

    /**
     * Closes and releases resources associated with the VisionPortal.
     */
    public void close() {
        visionPortal.close();
    }
}
