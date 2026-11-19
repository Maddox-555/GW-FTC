package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Autonomous(name = BioBuzzFullAuto16515.OP_MODE_NAME)
public class BioBuzzFullAuto16515 extends LinearOpMode {

    /** Name on the Driver Hub, also used as the op_mode label in the metrics database. */
    static final String OP_MODE_NAME = "BioBuzz Auto Test 16515";

    private RobotFacade robot;

    private AprilTagVisionSystem visionSystem;
    private ITagPoseData tagPoseData;

    private DcMotorEx flyWheel;
    private Servo lowerGate, upperGate;
    private DistanceSensor distanceSensor;

    private String alliance = "RED";

    private static final double UPPER_GATE_OPEN = 0.80;
    private static final double UPPER_GATE_CLOSED = 0.45;

    private static final double MAX_DRIVE_POWER = 0.18;
    private static final double FORWARD_GAIN = 0.015;
    private static final double STRAFE_GAIN = 0.025;
    private static final double TURN_GAIN = 0.010;
    private static final double BACK_UP_POWER = 0.10;

    // AUTO lasts 30 s and scoring needs about 7 s (2 s spin-up + 3 shots),
    // so stop trying to line up in time to still shoot.
    private static final long AUTO_LENGTH_MS = 30000;
    private static final long SCORING_TIME_MS = 7000;
    private static final long SAFETY_MARGIN_MS = 1500;
    private static final long ALIGN_TIMEOUT_MS =
            AUTO_LENGTH_MS - SCORING_TIME_MS - SAFETY_MARGIN_MS;

    // Stop backing up after this long (for example when the robot is against the wall)
    private static final long BACK_UP_TIMEOUT_MS = 2000;

    // A missed camera frame or two is normal; only give up on the target after this long
    private static final long LOST_TARGET_GRACE_MS = 300;

    // New camera frames in a row that must be "ready" before shooting
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

    private AutoState currentState = AutoState.SEARCHING;
    private long startTime = 0;
    private long stateStartTime = 0;
    private final long[] timeInState = new long[AutoState.values().length];

    private long lastSeenTime = 0;
    private long lastFrameTime = 0;
    private int readyFrames = 0;
    private ShotGeometry lastShot = null;

    // Metrics
    private long firstDetectionMs = -1;
    private int targetLostCount = 0;
    private int backUpTimeouts = 0;
    private int shotsFired = 0;
    private boolean summaryLogged = false;

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
                telemetry.addData("Aiming with", aimingMode());
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

            startTime = System.currentTimeMillis();
            stateStartTime = startTime;
            logMetric("BatteryVoltageStart", batteryVoltage());

            final double SEARCH_FORWARD_POWER = 0.06; // Slower forward search
            final double SEARCH_TURN_POWER = 0.08;
            final long SWEEP_INTERVAL_MS = 1500;

            while (opModeIsActive()) {
                long currentTime = System.currentTimeMillis();
                long totalElapsed = currentTime - startTime;

                // Out of time to line up and still shoot before AUTO ends
                boolean liningUp = currentState == AutoState.SEARCHING
                        || currentState == AutoState.BACKING_UP
                        || currentState == AutoState.ALIGNING;

                if (liningUp && totalElapsed > ALIGN_TIMEOUT_MS) {
                    changeState(AutoState.TIMEOUT, currentTime);
                }

                switch (currentState) {
                    case SEARCHING: {
                        List<AprilTagDetection> detections =
                                tagPoseData.getDetections();

                        AprilTagDetection target =
                                tagPoseData.findScoringTarget(alliance);

                        // If target seen, transition to backing up
                        if (target != null) {
                            if (firstDetectionMs < 0) {
                                firstDetectionMs = totalElapsed;
                            }
                            stopDrive();
                            lastSeenTime = currentTime;
                            changeState(AutoState.BACKING_UP, currentTime);
                            break;
                        }

                        // Go forward slowly and sweep turn until target is seen
                        double searchTurn =
                                ((totalElapsed / SWEEP_INTERVAL_MS) % 2 == 0)
                                        ? SEARCH_TURN_POWER
                                        : -SEARCH_TURN_POWER;

                        double forwardPower = SEARCH_FORWARD_POWER;
                        double distanceCm = readDistanceCm();

                        if (Double.isFinite(distanceCm) && distanceCm < 15.0) {
                            forwardPower = 0.0;
                        }

                        setDrivePowers(forwardPower, 0.0, searchTurn);

                        telemetry.addData("Status", "SEARCHING (Going forward slowly)");
                        telemetry.addData("Alliance", alliance);
                        telemetry.addData("Detections", detections.size());
                        telemetry.addData("Distance (cm)", distanceCm);
                        telemetry.addData("Elapsed ms", totalElapsed);
                        telemetry.update();
                        break;
                    }

                    case BACKING_UP: {
                        AprilTagDetection target =
                                tagPoseData.findScoringTarget(alliance);

                        if (target == null) {
                            handleLostTarget(currentTime);
                            break;
                        }
                        lastSeenTime = currentTime;

                        ShotGeometry shot = new ShotGeometry(target.ftcPose);
                        boolean tooClose = shot.tooCloseToMeasure
                                || (shot.valid && shot.distance < shot.desiredDistance);
                        boolean outOfTime = currentTime - stateStartTime > BACK_UP_TIMEOUT_MS;

                        // Back up until we reach the desired distance (or run out of time)
                        if (tooClose && !outOfTime) {
                            setDrivePowers(-BACK_UP_POWER, 0.0, 0.0);
                            telemetry.addData("Status", "BACKING_UP");
                            telemetry.addData("Distance (in)", shot.distance);
                            telemetry.addData("Desired (in)", shot.desiredDistance);
                            telemetry.update();
                        } else {
                            if (tooClose) {
                                backUpTimeouts++;
                            }
                            stopDrive();
                            readyFrames = 0;
                            changeState(AutoState.ALIGNING, currentTime);
                        }
                        break;
                    }

                    case ALIGNING: {
                        AprilTagDetection target =
                                tagPoseData.findScoringTarget(alliance);

                        if (target == null) {
                            handleLostTarget(currentTime);
                            break;
                        }
                        lastSeenTime = currentTime;

                        ShotGeometry shot = new ShotGeometry(target.ftcPose);
                        lastShot = shot;

                        // getDetections() repeats the last frame until the camera sends a new one
                        boolean newFrame = target.frameAcquisitionNanoTime != lastFrameTime;
                        lastFrameTime = target.frameAcquisitionNanoTime;

                        if (shot.ready()) {
                            stopDrive();
                            if (newFrame) {
                                readyFrames++;
                            }
                            if (readyFrames >= REQUIRED_READY_FRAMES) {
                                changeState(AutoState.READY_TO_SCORE, currentTime);
                            }
                        } else {
                            readyFrames = 0;

                            if (shot.tooCloseToMeasure) {
                                // Almost underneath the cell: back straight up until it can be measured
                                setDrivePowers(-BACK_UP_POWER, 0.0, 0.0);
                            } else if (!shot.valid) {
                                stopDrive();
                            } else {
                                double forward = clip(
                                        shot.distanceError() * FORWARD_GAIN,
                                        -MAX_DRIVE_POWER,
                                        MAX_DRIVE_POWER
                                );

                                double strafe = clip(
                                        shot.side * STRAFE_GAIN,
                                        -MAX_DRIVE_POWER,
                                        MAX_DRIVE_POWER
                                );

                                // headingDeg > 0 means the target is to the LEFT, but a positive
                                // turn in drive() turns RIGHT, so the sign is flipped here.
                                double turn = clip(
                                        -shot.headingDeg * TURN_GAIN,
                                        -MAX_DRIVE_POWER,
                                        MAX_DRIVE_POWER
                                );

                                setDrivePowers(forward, strafe, turn);
                            }
                        }

                        telemetry.addData("Status", "ALIGNING");
                        telemetry.addData("Alliance", alliance);
                        telemetry.addData("Target", AprilTagVisionSystem.targetName(target));
                        telemetry.addData("Aiming with", aimingMode());
                        telemetry.addData("Distance (in)", "%.1f (want %.1f)",
                                shot.distance, shot.desiredDistance);
                        telemetry.addData("Side (in)", "%.1f", shot.side);
                        telemetry.addData("Heading (deg)", "%.1f", shot.headingDeg);
                        telemetry.addData("Upright", shot.upright);
                        telemetry.addData("Ready frames", readyFrames);
                        if (shot.tooCloseToMeasure) {
                            telemetry.addLine("TOO CLOSE TO MEASURE - backing up");
                        }
                        if (shot.cameraMoved()) {
                            telemetry.addLine(String.format(
                                    "CAMERA MOVED? set to %.0f deg, seeing %.0f deg",
                                    RobotConstants.CAMERA_TILT_DEG, shot.measuredTiltDeg));
                        }
                        telemetry.update();
                        break;
                    }

                    case READY_TO_SCORE: {
                        stopDrive();
                        telemetry.addData("Status", "READY_TO_SCORE");
                        telemetry.update();

                        logMetric("AlignmentElapsedMs", totalElapsed);
                        logMetric("AlignmentSuccessful", 1.0);
                        logFinalAim();

                        changeState(AutoState.SCORING, currentTime);
                        break;
                    }

                    case SCORING: {
                        stopDrive();
                        telemetry.addData("Status", "SCORING (Spinning up flywheel & shooting)");
                        telemetry.update();

                        scoreThreeTimes();

                        changeState(AutoState.FINISHED, System.currentTimeMillis());
                        break;
                    }

                    case TIMEOUT: {
                        stopDrive();
                        telemetry.addData("Status", "TIMEOUT");
                        telemetry.update();

                        logMetric("AlignmentElapsedMs", totalElapsed);
                        logMetric("AlignmentSuccessful", 0.0);
                        logFinalAim();

                        telemetry.addLine("Scoring skipped: target not ready.");
                        telemetry.update();

                        changeState(AutoState.FINISHED, currentTime);
                        break;
                    }

                    case FINISHED: {
                        stopAllMotors();
                        logSummary();
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
            logSummary();  // also saves the metrics if AUTO was stopped early

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

        flyWheel.setDirection(RobotConstants.FLYWHEEL_DIRECTION);

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

            // Encoder ticks per second just before the ball goes in
            logMetric("FlywheelVelocityShot" + shot, flyWheel.getVelocity());

            upperGate.setPosition(UPPER_GATE_OPEN);
            sleep(250);

            upperGate.setPosition(UPPER_GATE_CLOSED);
            shotsFired++;

            if (shot < 3) {
                sleep(2000);
            }
        }

        flyWheel.setPower(0);
        upperGate.setPosition(UPPER_GATE_CLOSED);
    }

    /** Stops and waits briefly for the target to come back before going back to SEARCHING. */
    private void handleLostTarget(long now) {
        stopDrive();

        if (now - lastSeenTime > LOST_TARGET_GRACE_MS) {
            targetLostCount++;
            readyFrames = 0;
            changeState(AutoState.SEARCHING, now);
        } else {
            telemetry.addData("Status", currentState + " (target lost, waiting)");
            telemetry.update();
        }
    }

    /** Moves to the next state and adds up how long each state took. */
    private void changeState(AutoState next, long now) {
        timeInState[currentState.ordinal()] += now - stateStartTime;
        currentState = next;
        stateStartTime = now;
    }

    private String aimingMode() {
        return ShotGeometry.usesFloorGeometry()
                ? "floor distance (works at any camera tilt)"
                : "camera numbers (changes with camera tilt)";
    }

    /** Saves how far off the robot was the last time it saw the target. */
    private void logFinalAim() {
        if (lastShot == null) {
            return;
        }
        logMetric("FinalDistanceError", lastShot.distanceError());
        logMetric("FinalSideError", lastShot.side);
        logMetric("FinalHeadingErrorDeg", lastShot.headingDeg);
        logMetric("CameraTiltDeg", lastShot.measuredTiltDeg);
        logMetric("UsesFloorGeometry", ShotGeometry.usesFloorGeometry() ? 1.0 : 0.0);
    }

    /** Saves one summary of the run. Safe to call more than once. */
    private void logSummary() {
        if (summaryLogged || startTime == 0) {
            return;
        }
        summaryLogged = true;

        long now = System.currentTimeMillis();
        changeState(currentState, now);  // count the time in the current state

        if (firstDetectionMs >= 0) {
            logMetric("FirstDetectionMs", firstDetectionMs);
        }
        logMetric("TargetLostCount", targetLostCount);
        logMetric("BackUpTimeouts", backUpTimeouts);
        logMetric("SearchMs", timeInState[AutoState.SEARCHING.ordinal()]);
        logMetric("BackingUpMs", timeInState[AutoState.BACKING_UP.ordinal()]);
        logMetric("AligningMs", timeInState[AutoState.ALIGNING.ordinal()]);
        logMetric("ShotsFired", shotsFired);
        logMetric("AutoTotalMs", now - startTime);
        logMetric("BatteryVoltageEnd", batteryVoltage());
    }

    /** Records one metric. Skips values that aren't numbers, which the database can't store. */
    private void logMetric(String name, double value) {
        if (robot != null && Double.isFinite(value)) {
            robot.logMetric(OP_MODE_NAME, name, value);
        }
    }

    /** Lowest battery voltage reported by the hubs, or NaN if none. */
    private double batteryVoltage() {
        double lowest = Double.NaN;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double volts = sensor.getVoltage();
            if (volts > 0 && (Double.isNaN(lowest) || volts < lowest)) {
                lowest = volts;
            }
        }
        return lowest;
    }

    private double readDistanceCm() {
        return distanceSensor != null
                ? distanceSensor.getDistance(DistanceUnit.CM)
                : Double.NaN;
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
