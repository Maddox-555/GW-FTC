package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Factory class responsible for instantiating hardware subsystems such as the drive subsystem.
 * Follows the Factory design pattern to decouple subsystem creation from OpModes.
 */
public class HardwareFactory {

    /**
     * Private constructor to prevent instantiation of utility factory class.
     */
    private HardwareFactory() {
        // Utility class: do not create factory instances.
    }

    /**
     * Creates and returns a new HardwareDriveSubsystem instance using the provided HardwareMap.
     *
     * @param hardwareMap the FTC hardware map
     * @return initialized HardwareDriveSubsystem
     */
    public static HardwareDriveSubsystem createDriveSubsystem(
            HardwareMap hardwareMap) {

        return new HardwareDriveSubsystem(hardwareMap);
    }
}
