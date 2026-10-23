package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;

@TeleOp(name="BIOTELEOP1", group="Linear Opmode")
public class BIOTELEOP1 extends LinearOpMode {

    private RobotFacade robot;
    private DcMotorEx flyWheel;
    private Servo lowerGate, upperGate;
    private DistanceSensor distanceSensor;

    private static final double UPPER_GATE_OPEN = 0.80;
    private static final double UPPER_GATE_CLOSED = 0.5;

    @Override
    public void runOpMode() {
        robot = new RobotFacade(hardwareMap);

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        lowerGate = hardwareMap.get(Servo.class, "lowerGate");
        upperGate = hardwareMap.get(Servo.class, "upperGate");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        upperGate.setPosition(UPPER_GATE_CLOSED);

        flyWheel.setDirection(DcMotor.Direction.FORWARD);

        //Rev2mDistanceSensor revSensor = (Rev2mDistanceSensor) distanceSensor;

        waitForStart();

        while (opModeIsActive()) {
            double distanceCM = distanceSensor.getDistance(DistanceUnit.CM);
            double y  = -gamepad1.left_stick_y;   // forward/back
            double x  = gamepad1.left_stick_x * 1.05; // strafe (slight boost)
            double rx = gamepad1.right_stick_x * 0.5;   // rotate

            robot.drive(y, x, rx);

            //while (!gamepad1.left_bumper){
            //flywheel.setPower(0.5);
            //}
            //if (gamepad1.left_bumper){
            //  sleep;
            //}

            if (gamepad1.right_bumper){
                flyWheel.setPower(0.6);
            }
            else if(gamepad1.left_bumper){
                flyWheel.setPower(0.8);
            }
            else{
                flyWheel.setPower(0.0);
            }

            if (gamepad1.a) {
                upperGate.setPosition(UPPER_GATE_OPEN);

            }else if (gamepad1.b) {
                upperGate.setPosition(UPPER_GATE_OPEN);

            }else if (gamepad1.y) {

                upperGate.setPosition(UPPER_GATE_CLOSED);
            }else {

                upperGate.setPosition(UPPER_GATE_CLOSED);
            }

            //if(gamepad1.y){
            //    upperGate.setPosition(0.15);
            //}
            //else{
            //    upperGate.setPosition(0.5);
            //}

            //if(gamepad1.b){
            //    lowerGate.setPosition(0);
            //}
            //else{
            //    lowerGate.setPosition(0.25);
            //}

            telemetry.addData(
                    "FL/FR",
                    "%d / %d",
                    robot.getDriveSubsystem().getFrontLeftPosition(),
                    robot.getDriveSubsystem().getFrontRightPosition()
            );
            telemetry.addData(
                    "BL/BR",
                    "%d / %d",
                    robot.getDriveSubsystem().getBackLeftPosition(),
                    robot.getDriveSubsystem().getBackRightPosition()
            );
            telemetry.addData("lowerGate Position", lowerGate.getPosition());
            telemetry.addData("upperGate Position", upperGate.getPosition());
            telemetry.addData("Distance(cm)", distanceCM);
            telemetry.update();

            sleep(100);

            // todo: write your code here

        }
    }
}