package org.firstinspires.ftc.teamcode;

/**
 * Interface defining the motor control contract for the robot's drivetrain.
 * Implementations handle kinematics mixing (e.g., Mecanum drive) and motor power application.
 */
public interface IMotorControl {

    /**
     * Drives the robot according to normalized forward, strafe, and turn inputs.
     *
     * @param forward forward movement power [-1.0, 1.0]
     * @param strafe  lateral/strafe movement power [-1.0, 1.0]
     * @param turn    rotational turn power [-1.0, 1.0]
     */
    void drive(double forward, double strafe, double turn);

    /**
     * Immediately stops all motors controlled by this subsystem.
     */
    void stop();

}
