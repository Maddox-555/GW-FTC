package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = "BioBuzz Auto Test 16515")
public class BioBuzzFullAuto16515 extends LinearOpMode {

    private RobotFacade robot;

    private AprilTagVisionSystem visionSystem;
    private ITagPoseData tagPoseData;

    private DcMotorEx flyWheel;
    private Servo lowerGate, upperGate;
    private DistanceSensor distanceSensor;

    private String alliance = "RED";

    private static final double UPPER_GATE_OPEN = 0.80;
    private static final double UPPER_GATE_CLOSED = 0.45;

    private static final double DESIRED_DISTANCE = 24.0;
    private static final double DISTANCE_TOLERANCE = 2.0;
    private static final double X_TOLERANCE = 2.0;
    private static final double BEARING_TOLERANCE = 5.0;

    private static final double MAX_DRIVE_POWER = 0.18;
    private static final double FORWARD_GAIN = 0.015;
    private static final double STRAFE_GAIN = 0.025;
    private static final double TURN_GAIN = 0.010;

    private static final long ALIGN_TIMEOUT_MS = 12000;
    private static final int REQUIRED_READY_FRAMES = 5;

    private enum AutoState {
        SEARCHING,
        BACKING_UP,
        ALIGNING,
        READY_TO_SCORE,
        SCORING,
        TIMEOUT,
        FINISHED
    }

    @Override
    public void runOpMode() {
        try {
            initializeHardware();
            initializeVision();

            // Choose alliance before starting autonomous.
            while (!isStarted() && !isStopRequested()) {
                if (gamepad1.dpad_left) {
                    alliance = "RED";
                } else if (gamepad1.dpad_right) {
                    alliance = "BLUE";
                }

                telemetry.addLine("BioBuzz Full Autonomous");
                telemetry.addLine("D-pad LEFT: RED");
                telemetry.addLine("D-pad RIGHT: BLUE");
                telemetry.addData("Alliance", alliance);
                telemetry.addData(
                        "Camera",
                        visionSystem.getCameraState()
                );
                telemetry.addLine("Press PLAY to begin");
                telemetry.update();

                idle();
            }

            if (isStopRequested()) {
                return;
            }

            waitForStart();

            if (!opModeIsActive()) {
                return;
            }

            telemetry.addLine("AUTONOMOUS STARTED");
            telemetry.update();

            AutoState currentState = AutoState.SEARCHING;
            int readyFrames = 0;
            long startTime = System.currentTimeMillis();

            final double SEARCH_FORWARD_POWER = 0.06; // Slower forward search
            final double SEARCH_TURN_POWER = 0.08;
            final long SWEEP_INTERVAL_MS = 1500;

            while (opModeIsActive()) {
                long currentTime = System.currentTimeMillis();
                long totalElapsed = currentTime - startTime;

                // Check overall timeout
                if (currentState != AutoState.FINISHED
                        && currentState != AutoState.SCORING
                        && totalElapsed > ALIGN_TIMEOUT_MS) {
                    currentState = AutoState.TIMEOUT;
                }

                switch (currentState) {
                    case SEARCHING: {
                        List<AprilTagDetection> detections =
                                tagPoseData.getDetections();

                        AprilTagDetection target =
                                tagPoseData.findScoringTarget(alliance);

                        // If target seen, transition to backing up
                        if (target != null && target.ftcPose != null) {
                            stopDrive();
                            currentState = AutoState.BACKING_UP;
                            break;
                        }

                        // Go forward slowly and sweep turn until target is seen
                        double searchTurn =
                                ((totalElapsed / SWEEP_INTERVAL_MS) % 2 == 0)
                                        ? SEARCH_TURN_POWER
                                        : -SEARCH_TURN_POWER;

                        double forwardPower = SEARCH_FORWARD_POWER;

                        if (distanceSensor != null) {
                            double distanceCm =
                                    distanceSensor.getDistance(DistanceUnit.CM);

                            if (Double.isFinite(distanceCm)
                                    && distanceCm < 15.0) {
                                forwardPower = 0.0;
                            }
                        }

                        setDrivePowers(forwardPower, 0.0, searchTurn);

                        telemetry.addData("Status", "SEARCHING (Going forward slowly)");
                        telemetry.addData("Alliance", alliance);
                        telemetry.addData("Detections", detections.size());
                        telemetry.addData("Distance (cm)", distanceSensor != null ? distanceSensor.getDistance(DistanceUnit.CM) : 0.0);
                        telemetry.addData("Elapsed ms", totalElapsed);
                        telemetry.update();
                        break;
                    }

                    case BACKING_UP: {
                        AprilTagDetection target =
                                tagPoseData.findScoringTarget(alliance);

                        if (target == null || target.ftcPose == null) {
                            currentState = AutoState.SEARCHING;
                            break;
                        }

                        double y = target.ftcPose.y;

                        // Back up until we reach or exceed desired distance (e.g. 24 inches)
                        if (y < DESIRED_DISTANCE) {
                            // Move backward
                            setDrivePowers(-0.10, 0.0, 0.0);
                            telemetry.addData("Status", "BACKING_UP");
                            telemetry.addData("Current Y (in)", y);
                            telemetry.addData("Desired Y (in)", DESIRED_DISTANCE);
                            telemetry.update();
                        } else {
                            // Reached desired distance, stop and start aligning
                            stopDrive();
                            currentState = AutoState.ALIGNING;
                            readyFrames = 0;
                        }
                        break;
                    }

                    case ALIGNING: {
                        AprilTagDetection target =
                                tagPoseData.findScoringTarget(alliance);

                        if (target == null || target.ftcPose == null) {
                            currentState = AutoState.SEARCHING;
                            readyFrames = 0;
                            break;
                        }

                        double x = target.ftcPose.x;
                        double y = target.ftcPose.y;
                        double bearing = target.ftcPose.bearing;
                        double roll = target.ftcPose.roll;

                        boolean upright = Math.abs(roll) < 90.0;

                        boolean correctDistance =
                                Math.abs(y - DESIRED_DISTANCE)
                                        <= DISTANCE_TOLERANCE;

                        boolean centered =
                                Math.abs(x) <= X_TOLERANCE;

                        boolean aimed =
                                Math.abs(bearing) <= BEARING_TOLERANCE;

                        boolean targetReady =
                                upright && correctDistance && centered && aimed;

                        if (targetReady) {
                            readyFrames++;
                            stopDrive();
                            if (readyFrames >= REQUIRED_READY_FRAMES) {
                                currentState = AutoState.READY_TO_SCORE;
                            }
                        } else {
                            readyFrames = 0;

                            double forward = clip(
                                    (y - DESIRED_DISTANCE) * FORWARD_GAIN,
                                    -MAX_DRIVE_POWER,
                                    MAX_DRIVE_POWER
                            );

                            double strafe = clip(
                                    x * STRAFE_GAIN,
                                    -MAX_DRIVE_POWER,
                                    MAX_DRIVE_POWER
                            );

                            double turn = clip(
                                    bearing * TURN_GAIN,
                                    -MAX_DRIVE_POWER,
                                    MAX_DRIVE_POWER
                            );

                            setDrivePowers(forward, strafe, turn);
                        }

                        telemetry.addData("Status", "ALIGNING");
                        telemetry.addData("Alliance", alliance);
                        telemetry.addData("X (in)", x);
                        telemetry.addData("Y (in)", y);
                        telemetry.addData("Bearing (deg)", bearing);
                        telemetry.addData("Roll (deg)", roll);
                        telemetry.addData("Ready frames", readyFrames);
                        telemetry.update();
                        break;
                    }

                    case READY_TO_SCORE: {
                        stopDrive();
                        telemetry.addData("Status", "READY_TO_SCORE");
                        telemetry.update();

                        robot.logMetric(
                                "BioBuzzFullAuto16515",
                                "AlignmentElapsedMs",
                                totalElapsed
                        );
                        robot.logMetric(
                                "BioBuzzFullAuto16515",
                                "AlignmentSuccessful",
                                1.0
                        );

                        currentState = AutoState.SCORING;
                        break;
                    }

                    case SCORING: {
                        stopDrive();
                        telemetry.addData("Status", "SCORING (Spinning up flywheel & shooting)");
                        telemetry.update();

                        scoreThreeTimes();

                        currentState = AutoState.FINISHED;
                        break;
                    }

                    case TIMEOUT: {
                        stopDrive();
                        telemetry.addData("Status", "TIMEOUT");
                        telemetry.update();

                        robot.logMetric(
                                "BioBuzzFullAuto16515",
                                "AlignmentElapsedMs",
                                totalElapsed
                        );
                        robot.logMetric(
                                "BioBuzzFullAuto16515",
                                "AlignmentSuccessful",
                                0.0
                        );

                        telemetry.addLine("Scoring skipped: target not ready.");
                        telemetry.update();

                        currentState = AutoState.FINISHED;
                        break;
                    }

                    case FINISHED: {
                        stopAllMotors();
                        telemetry.addData("Status", "FINISHED");
                        telemetry.addLine("Autonomous sequence finished.");
                        telemetry.update();
                        return;
                    }
                }

                sleep(20);
            }

        } finally {
            stopAllMotors();

            if (visionSystem != null) {
                visionSystem.close();
            }
        }
    }

    private void initializeHardware() {
        robot = new RobotFacade(hardwareMap);

        flyWheel = hardwareMap.get(
                DcMotorEx.class, "flyWheel"
        );

        lowerGate = hardwareMap.get(
                Servo.class, "lowerGate"
        );

        upperGate = hardwareMap.get(
                Servo.class, "upperGate"
        );

        distanceSensor = hardwareMap.get(
                DistanceSensor.class, "distanceSensor"
        );

        flyWheel.setDirection(DcMotor.Direction.REVERSE);

        upperGate.setPosition(UPPER_GATE_CLOSED);
        lowerGate.setPosition(0.25);

        flyWheel.setPower(0);
    }

    private void initializeVision() {
        visionSystem = new AprilTagVisionSystem(hardwareMap);
        tagPoseData = visionSystem;
    }

    private void scoreThreeTimes() {
        // Spin up flywheel first and wait 2 seconds for it to reach full speed before shooting
        flyWheel.setPower(0.65);
        telemetry.addLine("Flywheel spinning up...");
        telemetry.update();
        sleep(2000);

        for (int shot = 1;
             shot <= 3 && opModeIsActive();
             shot++) {

            telemetry.addData("Scoring shot", "%d / 3", shot);
            telemetry.update();

            upperGate.setPosition(UPPER_GATE_OPEN);
            sleep(250);

            upperGate.setPosition(UPPER_GATE_CLOSED);

            if (shot < 3) {
                sleep(2000);
            }
        }

        flyWheel.setPower(0);
        upperGate.setPosition(UPPER_GATE_CLOSED);
    }

    private void setDrivePowers(
            double forward,
            double strafe,
            double turn) {
        robot.drive(forward, strafe, turn);
    }

    private void stopDrive() {
        if (robot != null) {
            robot.stopDrive();
        }
    }

    private void stopAllMotors() {
        stopDrive();

        if (flyWheel != null) {
            flyWheel.setPower(0.0);
        }

        if (upperGate != null) {
            upperGate.setPosition(UPPER_GATE_CLOSED);
        }
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
