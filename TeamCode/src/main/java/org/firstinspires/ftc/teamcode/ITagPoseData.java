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
     * Finds the alliance HIVE cell to shoot at (the SCORING or AUDIENCE cluster, whichever is seen).
     *
     * @param alliance the alliance color string ("RED" or "BLUE")
     * @return the matching AprilTagDetection with a pose, or null if not found
     */
    AprilTagDetection findScoringTarget(String alliance);
}
