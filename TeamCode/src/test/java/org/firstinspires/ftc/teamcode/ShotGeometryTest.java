package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assume.assumeTrue;

import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.junit.Test;

/**
 * Checks the ShotGeometry math on the laptop. Run with:
 *   ./gradlew :TeamCode:testDebugUnitTest --offline
 *
 * Each test places a cluster at a known floor distance and height, then works out
 * what a camera tilted up by some angle would report (ftcPose y and z), and checks
 * that ShotGeometry gets the floor numbers back.
 */
public class ShotGeometryTest {

    private static final double TOLERANCE = 1e-9;

    /** ftcPose.y a camera tilted up by tiltDeg reports for a cluster floorForward ahead and height up. */
    private static double cameraY(double floorForward, double height, double tiltDeg) {
        double tilt = Math.toRadians(tiltDeg);
        return floorForward * Math.cos(tilt) + height * Math.sin(tilt);
    }

    /** ftcPose.z for the same cluster and camera. */
    private static double cameraZ(double floorForward, double height, double tiltDeg) {
        double tilt = Math.toRadians(tiltDeg);
        return -floorForward * Math.sin(tilt) + height * Math.cos(tilt);
    }

    @Test
    public void floorDistanceIsTheSameAtEveryCameraTilt() {
        double floorForward = 30.0;
        double height = 28.0;

        for (double tilt : new double[] {10, 20, 30, 40, 50}) {
            double y = cameraY(floorForward, height, tilt);
            double z = cameraZ(floorForward, height, tilt);

            assertEquals("tilt " + tilt, floorForward,
                    ShotGeometry.floorForward(y, z, height), TOLERANCE);
        }
    }

    @Test
    public void cameraYChangesWithTiltEvenThoughTheRobotDidNotMove() {
        // This is why the old code parked in different spots when the camera moved
        double y20 = cameraY(30.0, 28.0, 20);
        double y40 = cameraY(30.0, 28.0, 40);

        assertTrue(Math.abs(y20 - y40) > 3.0);
    }

    @Test
    public void tiltIsWorkedOutFromTheTags() {
        double height = 28.0;

        for (double tilt : new double[] {15, 25, 35, 45}) {
            double y = cameraY(24.0, height, tilt);
            double z = cameraZ(24.0, height, tilt);

            assertEquals(tilt, ShotGeometry.tiltDeg(y, z, height), 1e-6);
        }
    }

    @Test
    public void heightIsWorkedOutFromTheTilt() {
        for (double height : new double[] {20, 28, 36}) {
            double y = cameraY(40.0, height, 30);
            double z = cameraZ(40.0, height, 30);

            assertEquals(height, ShotGeometry.heightFromTilt(y, z, 30), TOLERANCE);
        }
    }

    @Test
    public void impossibleNumbersGiveNaN() {
        // The cluster can't be closer than its own height
        assertTrue(Double.isNaN(ShotGeometry.floorForward(10.0, 5.0, 30.0)));
        assertTrue(Double.isNaN(ShotGeometry.tiltDeg(10.0, 5.0, 30.0)));
    }

    @Test
    public void beforeMeasuringItUsesTheCameraNumbers() {
        // Only checked while RobotConstants still has 0 (not measured) for everything
        assumeTrue(nothingMeasuredYet());
        assertFalse(ShotGeometry.usesFloorGeometry());

        // x = 3 in to the right, y = 25 in ahead, bearing = -6.8 deg
        AprilTagPoseFtc pose = new AprilTagPoseFtc(3.0, 25.0, 10.0, 0, 0, 5.0,
                Math.hypot(3.0, 25.0), Math.toDegrees(Math.atan2(-3.0, 25.0)), 0);
        ShotGeometry shot = new ShotGeometry(pose);

        assertTrue(shot.valid);
        assertEquals(25.0, shot.distance, TOLERANCE);
        assertEquals(3.0, shot.side, TOLERANCE);
        assertEquals(Math.toDegrees(Math.atan2(-3.0, 25.0)), shot.headingDeg, TOLERANCE);
        assertEquals(RobotConstants.DESIRED_CAMERA_DISTANCE, shot.desiredDistance, TOLERANCE);
        assertEquals(Math.sqrt(9 + 625 + 100), shot.range3d, TOLERANCE);
        assertTrue(Double.isNaN(shot.floorDistance));
        assertTrue(Double.isNaN(shot.heightFromTilt));
    }

    @Test
    public void readyNeedsDistanceSideHeadingAndUpright() {
        assumeTrue(nothingMeasuredYet());

        AprilTagPoseFtc ready = new AprilTagPoseFtc(0.5, 24.5, 10.0, 0, 0, 3.0,
                0, -1.0, 0);
        AprilTagPoseFtc upsideDown = new AprilTagPoseFtc(0.5, 24.5, 10.0, 0, 0, 175.0,
                0, -1.0, 0);
        AprilTagPoseFtc tooFar = new AprilTagPoseFtc(0.5, 30.0, 10.0, 0, 0, 3.0,
                0, -1.0, 0);
        AprilTagPoseFtc offToTheSide = new AprilTagPoseFtc(4.0, 24.0, 10.0, 0, 0, 3.0,
                0, -9.0, 0);

        assertTrue(new ShotGeometry(ready).ready());
        assertFalse(new ShotGeometry(upsideDown).ready());
        assertFalse(new ShotGeometry(tooFar).ready());
        assertFalse(new ShotGeometry(offToTheSide).ready());
    }

    private static boolean nothingMeasuredYet() {
        return RobotConstants.CAMERA_TILT_DEG == 0
                && RobotConstants.CLUSTER_HEIGHT_ABOVE_CAMERA == 0
                && RobotConstants.DESIRED_FLOOR_DISTANCE == 0;
    }
}
