package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "BioBuzz Auto Test 16515")
public class BioBuzzFullAuto16515 extends LinearOpMode {

    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private DcMotorEx flyWheel;
    private Servo lowerGate, upperGate;
    private DistanceSensor distanceSensor;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private String alliance = "RED";

    private static final double UPPER_GATE_OPEN = 0.80;
    private static final double UPPER_GATE_CLOSED = 0.45;

    private static final double DESIRED_DISTANCE = 24.0;
    private static final double DISTANCE_TOLERANCE = 2.0;
    private static final double X_TOLERANCE = 2.0;
    private static final double BEARING_TOLERANCE = 5.0;

    // Conservative initial values. Tune on the real robot.
    private static final double MAX_DRIVE_POWER = 0.18;
    private static final double FORWARD_GAIN = 0.015;
    private static final double STRAFE_GAIN = 0.025;
    private static final double TURN_GAIN = 0.010;

    private static final long ALIGN_TIMEOUT_MS = 8000;
    private static final int REQUIRED_READY_FRAMES = 5;

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
                telemetry.addData("Camera", visionPortal.getCameraState());
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

            boolean aligned = alignToScoringTarget();

            stopDrive();

            if (opModeIsActive() && aligned) {
                telemetry.addLine("Target confirmed. Starting score sequence.");
                telemetry.update();

                scoreThreeTimes();
            } else {
                telemetry.addLine("Scoring skipped: target not ready.");
                telemetry.update();
            }

            stopAllMotors();

            telemetry.addLine("Autonomous sequence finished.");
            telemetry.update();

            // Parking movement will be added after its distance and
            // direction are confirmed for the actual starting position.

        } finally {
            stopAllMotors();

            if (visionPortal != null) {
                visionPortal.close();
            }
        }
    }

    private void initializeHardware() {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");

        lowerGate = hardwareMap.get(Servo.class, "lowerGate");
        upperGate = hardwareMap.get(Servo.class, "upperGate");

        distanceSensor =
                hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // Preserve the motor directions used in the existing autonomous.
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        flyWheel.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotorEx motor : driveMotors()) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(0);
        }

        upperGate.setPosition(UPPER_GATE_CLOSED);
        lowerGate.setPosition(0.25);

        flyWheel.setPower(0);
    }

    private void initializeVision() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
    }

    private boolean alignToScoringTarget() {

        long startTime = System.currentTimeMillis();
        int readyFrames = 0;

        while (opModeIsActive()
                && System.currentTimeMillis() - startTime < ALIGN_TIMEOUT_MS) {

            AprilTagDetection target =
                    findScoringTarget(aprilTag.getDetections());

            if (target == null || target.ftcPose == null) {
                stopDrive();
                readyFrames = 0;

                telemetry.addData("Alliance", alliance);
                telemetry.addLine("Searching for scoring cluster...");
                telemetry.addData("Elapsed", "%.1f s",
                        (System.currentTimeMillis() - startTime) / 1000.0);
                telemetry.update();

                sleep(20);
                continue;
            }

            double x = target.ftcPose.x;
            double y = target.ftcPose.y;
            double bearing = target.ftcPose.bearing;
            double roll = target.ftcPose.roll;

            boolean upright = Math.abs(roll) < 90.0;
            boolean correctDistance =
                    Math.abs(y - DESIRED_DISTANCE) <= DISTANCE_TOLERANCE;
            boolean centered = Math.abs(x) <= X_TOLERANCE;
            boolean aimed = Math.abs(bearing) <= BEARING_TOLERANCE;

            boolean targetReady =
                    upright && correctDistance && centered && aimed;

            if (targetReady) {
                readyFrames++;
                stopDrive();
            } else {
                readyFrames = 0;

                // Proportional correction with conservative power limits.
                double forward = clip(
                        (y - DESIRED_DISTANCE) * FORWARD_GAIN,
                        -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

                double strafe = clip(
                        x * STRAFE_GAIN,
                        -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

                double turn = clip(
                        bearing * TURN_GAIN,
                        -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

                // Mecanum mixing; verify the correction directions
                // with the wheels raised before a floor test.
                setDrivePowers(forward, strafe, turn);
            }

            telemetry.addData("Alliance", alliance);
            String targetName = "Cluster";
            if (target instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) target;
                if (cluster.metadata != null && cluster.metadata.name != null) {
                    targetName = cluster.metadata.name;
                }
            }
            telemetry.addData("Target", targetName);
            telemetry.addData("X", "%.1f in", x);
            telemetry.addData("Y", "%.1f in", y);
            telemetry.addData("Bearing", "%.1f deg", bearing);
            telemetry.addData("Roll", "%.1f deg", roll);
            telemetry.addData("Upright", upright);
            telemetry.addData("Distance OK", correctDistance);
            telemetry.addData("Centered", centered);
            telemetry.addData("Aimed", aimed);
            telemetry.addData("Ready frames",
                    "%d / %d", readyFrames, REQUIRED_READY_FRAMES);
            telemetry.addData("Distance sensor",
                    "%.1f cm", distanceSensor.getDistance(DistanceUnit.CM));
            telemetry.update();

            if (readyFrames >= REQUIRED_READY_FRAMES) {
                stopDrive();
                return true;
            }

            sleep(20);
        }

        stopDrive();
        return false;
    }

    private AprilTagDetection findScoringTarget(
            List<AprilTagDetection> detections) {

        for (AprilTagDetection detection : detections) {

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

    private void scoreThreeTimes() {

        // Use the existing autonomous flywheel and upper-gate sequence.
        flyWheel.setPower(0.65);
        sleep(2000);

        for (int shot = 1; shot <= 3 && opModeIsActive(); shot++) {

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
            double forward, double strafe, double turn) {

        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(bl)),
                Math.max(Math.abs(fr), Math.abs(br))));

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    private void stopDrive() {
        for (DcMotorEx motor : driveMotors()) {
            motor.setPower(0);
        }
    }

    private void stopAllMotors() {
        stopDrive();

        if (flyWheel != null) {
            flyWheel.setPower(0);
        }

        if (upperGate != null) {
            upperGate.setPosition(UPPER_GATE_CLOSED);
        }
    }

    private DcMotorEx[] driveMotors() {
        return new DcMotorEx[] {
                frontLeft, frontRight, backLeft, backRight
        };
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
