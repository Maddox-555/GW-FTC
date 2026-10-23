
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareDriveSubsystem implements IMotorControl {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    public HardwareDriveSubsystem(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotorEx motor : getMotors()) {
            motor.setZeroPowerBehavior(
                    DcMotor.ZeroPowerBehavior.BRAKE
            );
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPower(0);
        }
    }

    @Override
    public void drive(double forward, double strafe, double turn) {

        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(fl), Math.abs(bl)),
                        Math.max(Math.abs(fr), Math.abs(br))
                )
        );

        frontLeft.setPower(fl / max);
        backLeft.setPower(bl / max);
        frontRight.setPower(fr / max);
        backRight.setPower(br / max);
    }

    @Override
    public void stop() {
        for (DcMotorEx motor : getMotors()) {
            motor.setPower(0);
        }
    }

    public int getFrontLeftPosition() {
        return frontLeft.getCurrentPosition();
    }

    public int getFrontRightPosition() {
        return frontRight.getCurrentPosition();
    }

    public int getBackLeftPosition() {
        return backLeft.getCurrentPosition();
    }

    public int getBackRightPosition() {
        return backRight.getCurrentPosition();
    }

    private DcMotorEx[] getMotors() {
        return new DcMotorEx[] {
                frontLeft, frontRight, backLeft, backRight
        };
    }
}
