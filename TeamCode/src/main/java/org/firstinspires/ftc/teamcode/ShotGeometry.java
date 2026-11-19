package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

/**
 * Turns a HIVE cluster's ftcPose into "how far away" and "which way to turn" for the shooter.
 *
 * The SDK's x / y / bearing are measured along the camera's own line of sight, so they
 * change whenever the camera tilt changes. Tilting the camera up or down changes y and z
 * but not (y² + z²), and not x. So once the cluster's height above the camera is known,
 * the floor distance and heading can be worked out for any tilt.
 *
 * Until RobotConstants.CLUSTER_HEIGHT_ABOVE_CAMERA and DESIRED_FLOOR_DISTANCE are measured,
 * aiming uses the camera's own numbers (the original behavior).
 */
public final class ShotGeometry {

    /** True when aiming uses floor distance and heading instead of the camera's own numbers. */
    public static boolean usesFloorGeometry() {
        return RobotConstants.CLUSTER_HEIGHT_ABOVE_CAMERA > 0
                && RobotConstants.DESIRED_FLOOR_DISTANCE > 0;
    }

    // ---- Used for aiming (floor values once measured, camera values before) ----

    /** False when this frame's numbers can't be used for aiming. */
    public final boolean valid;
    /**
     * True when the floor math has no answer because the cluster looks closer than its own
     * height (robot almost underneath it, or the height is measured a bit too high).
     * The robot should back up.
     */
    public final boolean tooCloseToMeasure;
    /** Inches from the shooter to the cluster. */
    public final double distance;
    /** What distance should be when ready to shoot. */
    public final double desiredDistance;
    /** Inches; + means the target is to the right. */
    public final double side;
    /** Degrees; + means the target is to the left (the robot should turn left). */
    public final double headingDeg;
    /** False when the cluster looks upside down. */
    public final boolean upright;

    // ---- For measuring and checking the camera ----

    /** Straight-line inches from the lens to the cluster. Camera tilt doesn't change it. */
    public final double range3d;
    /** Floor inches from the shooter to the cluster (NaN until the cluster height is measured). */
    public final double floorDistance;
    /** Camera tilt worked out from this frame (NaN until the cluster height is measured). */
    public final double measuredTiltDeg;
    /** Cluster height above the camera, worked out from CAMERA_TILT_DEG (NaN until the tilt is measured). */
    public final double heightFromTilt;

    public ShotGeometry(AprilTagPoseFtc pose) {
        double height = RobotConstants.CLUSTER_HEIGHT_ABOVE_CAMERA;

        upright = Math.abs(pose.roll) < 90.0;
        range3d = Math.sqrt(pose.x * pose.x + pose.y * pose.y + pose.z * pose.z);

        heightFromTilt = RobotConstants.CAMERA_TILT_DEG > 0
                ? heightFromTilt(pose.y, pose.z, RobotConstants.CAMERA_TILT_DEG)
                : Double.NaN;

        double cameraForward = height > 0 ? floorForward(pose.y, pose.z, height) : Double.NaN;
        measuredTiltDeg = Double.isNaN(cameraForward)
                ? Double.NaN
                : tiltDeg(pose.y, pose.z, height);

        double floorSide = pose.x + RobotConstants.CAMERA_RIGHT_OF_SHOOTER;
        double floorAhead = cameraForward + RobotConstants.CAMERA_AHEAD_OF_SHOOTER;
        floorDistance = Double.isNaN(cameraForward) ? Double.NaN : Math.hypot(floorAhead, floorSide);

        if (usesFloorGeometry()) {
            valid = !Double.isNaN(floorDistance);
            tooCloseToMeasure = !valid;
            distance = valid ? floorDistance : 0.0;
            side = floorSide;
            headingDeg = valid ? Math.toDegrees(Math.atan2(-floorSide, floorAhead)) : 0.0;
            desiredDistance = RobotConstants.DESIRED_FLOOR_DISTANCE;
        } else {
            valid = true;
            tooCloseToMeasure = false;
            distance = pose.y;
            side = pose.x;
            headingDeg = pose.bearing;
            desiredDistance = RobotConstants.DESIRED_CAMERA_DISTANCE;
        }
    }

    public double distanceError() {
        return distance - desiredDistance;
    }

    public boolean correctDistance() {
        return Math.abs(distanceError()) <= RobotConstants.DISTANCE_TOLERANCE;
    }

    public boolean centered() {
        return Math.abs(side) <= RobotConstants.SIDE_TOLERANCE;
    }

    public boolean aimed() {
        return Math.abs(headingDeg) <= RobotConstants.HEADING_TOLERANCE_DEG;
    }

    public boolean ready() {
        return valid && upright && correctDistance() && centered() && aimed();
    }

    /** True when the camera looks bumped compared to RobotConstants.CAMERA_TILT_DEG. */
    public boolean cameraMoved() {
        return RobotConstants.CAMERA_TILT_DEG > 0
                && !Double.isNaN(measuredTiltDeg)
                && Math.abs(measuredTiltDeg - RobotConstants.CAMERA_TILT_DEG)
                        > RobotConstants.TILT_WARNING_DEG;
    }

    // ---- The math (static so it can be tested on the laptop) ----
    // y and z are ftcPose.y and ftcPose.z (camera forward and camera up), in inches.

    /** Floor distance straight ahead of the camera to the point under the cluster, or NaN if impossible. */
    static double floorForward(double y, double z, double height) {
        double squared = y * y + z * z - height * height;
        return squared > 0 ? Math.sqrt(squared) : Double.NaN;
    }

    /** Camera tilt up from level, in degrees, given the cluster's height above the camera. */
    static double tiltDeg(double y, double z, double height) {
        double forward = floorForward(y, z, height);
        if (Double.isNaN(forward)) {
            return Double.NaN;
        }
        return Math.toDegrees(Math.atan2(height, forward) - Math.atan2(z, y));
    }

    /** Cluster height above the camera, given the camera tilt up from level in degrees. */
    static double heightFromTilt(double y, double z, double tiltDeg) {
        double tilt = Math.toRadians(tiltDeg);
        return y * Math.sin(tilt) + z * Math.cos(tilt);
    }
}
