package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

/**
 * Interface defining the vision telemetry data provider contract.
 * Provides access to raw AprilTag detections and alliance-specific scoring targets.
 */
public interface ITagPoseData {

    /**
     * Returns a list of all currently detected AprilTag detections.
     *
     * @return list of AprilTagDetection objects
     */
    List<AprilTagDetection> getDetections();

    /**
     * Finds and returns the specific AprilTag detection matching the alliance scoring target.
     *
     * @param alliance the alliance color string ("RED" or "BLUE")
     * @return the matching AprilTagDetection, or null if not found
     */
    AprilTagDetection findScoringTarget(String alliance);
}
