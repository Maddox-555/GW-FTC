
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class HardwareFactory {

    private HardwareFactory() {
        // Utility class: do not create factory instances.
    }

    public static HardwareDriveSubsystem createDriveSubsystem(
            HardwareMap hardwareMap) {

        return new HardwareDriveSubsystem(hardwareMap);
    }
}
