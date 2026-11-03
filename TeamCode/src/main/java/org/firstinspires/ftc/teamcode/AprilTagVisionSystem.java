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
                        hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    @Override
    public List<AprilTagDetection> getDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Searches through current detections and returns the scoring target matching the given alliance.
     *
     * @param alliance alliance string ("RED" or "BLUE")
     * @return AprilTagDetection matching the alliance scoring cluster, or null if not found
     */
    @Override
    public AprilTagDetection findScoringTarget(String alliance) {
        for (AprilTagDetection detection : getDetections()) {

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

            if (name.equalsIgnoreCase(alliance + " SCORING")) {
                return detection;
            }
        }

        return null;
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
