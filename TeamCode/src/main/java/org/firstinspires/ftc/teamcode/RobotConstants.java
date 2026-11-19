package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Numbers that more than one OpMode uses, kept in one place so they can't drift apart.
 *
 * Values in the "measure on the robot / field" section start at 0, which means
 * "not measured yet". Until they're filled in, aiming uses the camera's own numbers.
 */
public final class RobotConstants {

    private RobotConstants() {
        // Constants only
    }

    // ---- Hardware ----

    /** Webcam name in the Driver Hub configuration (see the README hardware map). */
    public static final String WEBCAM_NAME = "Webcam 1";

    /**
     * Flywheel direction for positive power. FORWARD launches correctly in BIOTELEOP1
     * (checked 2026-11-17). If balls ever launch backwards, change only this line.
     */
    public static final DcMotor.Direction FLYWHEEL_DIRECTION = DcMotor.Direction.FORWARD;

    // ---- Shooting position tolerances ----

    /** Camera "y" to the HIVE cluster when ready to shoot. Used only until the floor values below are measured. */
    public static final double DESIRED_CAMERA_DISTANCE = 24.0;  // inches

    public static final double DISTANCE_TOLERANCE = 2.0;       // inches
    public static final double SIDE_TOLERANCE = 2.0;           // inches
    public static final double HEADING_TOLERANCE_DEG = 5.0;    // degrees

    // ---- Measure on the robot / field (0 = not measured yet) ----

    /**
     * How far the camera is tilted up from level, in degrees (phone level app on the camera).
     * Must be more than 0: the HIVE tags face down, so the camera has to point up to see them.
     * At 0 or below, the camera-moved warning and the height read-out stay off.
     */
    public static final double CAMERA_TILT_DEG = 0.0;

    /** Height of the HIVE tag cluster above the camera lens, in inches (read it from the Hive test). */
    public static final double CLUSTER_HEIGHT_ABOVE_CAMERA = 0.0;

    /** Floor distance from the shooter to the cluster when shots go in, in inches (read it from the Hive test). */
    public static final double DESIRED_FLOOR_DISTANCE = 0.0;

    /** Camera lens position compared to where the ball leaves the shooter, in inches. */
    public static final double CAMERA_AHEAD_OF_SHOOTER = 0.0;  // + = camera is in front of the shooter
    public static final double CAMERA_RIGHT_OF_SHOOTER = 0.0;  // + = camera is to the right of the shooter

    /** Warn the drivers when the tilt seen through the camera differs from CAMERA_TILT_DEG by more than this. */
    public static final double TILT_WARNING_DEG = 3.0;
}
