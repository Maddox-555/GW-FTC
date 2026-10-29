package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagVisionSystem implements ITagPoseData {

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

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

    public String getCameraState() {
        return visionPortal.getCameraState().toString();
    }

    public void close() {
        visionPortal.close();
    }
}
