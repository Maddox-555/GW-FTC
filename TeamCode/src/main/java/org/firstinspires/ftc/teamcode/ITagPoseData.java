package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;

public interface ITagPoseData {
    List<AprilTagDetection> getDetections();

    AprilTagDetection findScoringTarget(String alliance);
}