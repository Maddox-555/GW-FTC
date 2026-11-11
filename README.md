# GW-FTC: BIOBUZZ (2026-2027 Season)

## Getting Started (Developer Setup)

To contribute to this codebase, ensure your development environment matches the FTC SDK v12.0 requirements:

1. **Android Studio:** Install **Android Studio Narwhal 3 Feature Drop** (or newer).
2. **Clone the Repo:** `git clone https://github.com/Maddox-555/GW-FTC.git`
3. **Gradle JDK:** Go to Settings -> Build, Execution, Deployment -> Build Tools -> Gradle. Set the **Gradle JDK** to `jbr-25 JetBrains Runtime`. 
4. Sync the project with Gradle files before running.

## Robot Setup & Hub Connection

### Hardware Power-On
1. Ensure the robot battery is fully charged and securely plugged into the REV Control Hub.
2. Turn on the main robot power switch. The Control Hub LED should transition from blue to blinking green.
3. Plug the Logitech C920X Webcam into the Control Hub's USB port.

### Connecting the Driver Station
1. Power on the REV Driver Hub.
2. Open the Wi-Fi settings and connect to the robot's network (SSID: `16515-RC`, Password: `Greenwood`).
3. Open the **FTC Driver Station** app.
4. Verify the camera, battery voltage, and ping status are green in the top right corner.
5. On the DualShock 4 controllers, press **Options + Cross (X)** to map Gamepad 1, and **Options + Circle (O)** to map Gamepad 2.

## Hardware Configuration Map

For the code to run correctly, the active configuration file on the REV Control Hub MUST match the names listed below exactly:

| Hardware Type | Hub Port | Configuration Name | Description |
| :--- | :--- | :--- | :--- |
| Drive Motor | Port 0 | `frontLeft` | Front-left mecanum drive motor |
| Drive Motor | Port 1 | `frontRight` | Front-right mecanum drive motor |
| Drive Motor | Port 2 | `backLeft` | Back-left mecanum drive motor |
| Drive Motor | Port 3 | `backRight` | Back-right mecanum drive motor |
| Webcam | USB | `Webcam 1` | Logitech C920X Pro HD |
| [Add Servo] | [Port] | `[name]` | [Description] |

## Software Architecture & Features

This project utilizes custom object-oriented design patterns to keep our code clean, modular, and easy to maintain.

* **`RobotFacade`:** Provides a single, simplified interface for controlling complex subsystems (drivetrain, vision, logging) so OpModes remain clean and readable.
* **`RobotMatchManager`:** A state machine that safely transitions the robot between Disabled, Autonomous, and TeleOp modes while monitoring for controller disconnects.
* **Metrics Database:** A local SQLite database on the Control Hub that logs match cycle times, battery voltage drops, and error telemetry for post-match analysis.

### Autonomous (`BioBuzzFullAuto16515`)
Our autonomous routine relies on the Logitech C920X webcam and the FTC AprilTag Processor to dynamically navigate the field. 
1. The robot scans for BIOBUZZ AprilTag clusters to determine the correct scoring target.
2. It calculates 6DOF pose data (X, Y, Z, Yaw) to align itself automatically.
3. [Add details about what the robot physically does, e.g., scores pollen, parks in the designated zone].

### TeleOp (`BIOTELEOP1`)
Driver-controlled mode utilizing field-centric mecanum driving. 
* **Gamepad 1:** Controls drivetrain movement (Left Stick: Translate, Right Stick: Rotate).
* **Gamepad 2:** Controls scoring mechanisms (intake, arm, launcher).

## Spring Maintenance & Future Development

For the Greenwood High School team taking over development for the remainder of the season, here is where to look when tuning or expanding the robot:

1. **Adjusting Field Measurements & Target IDs:** 
   If the physical field layout changes or you need to update which AprilTags to target, do **not** dig through the autonomous logic. Open `FieldConstants.java`—all hardcoded values are centralized there for easy updating.
2. **Tuning Autonomous Speed & Precision:**
   If the robot overshoots its target during autonomous, open `HardwareDriveSubsystem.java` to adjust the PID control coefficients, or adjust the autonomous delay settings in the Driver Station configuration menu.
3. **Adding New Scoring Routines:**
   If you add new physical mechanisms to the robot, instantiate them in `HardwareFactory.java` first, then create simple command methods (like `scorePollen()`) inside `RobotFacade.java` so your drivers can trigger them easily in TeleOp.

## WKU Contributors
* Maddox Simpson
* Chad Wilson
* Nb Bakari
