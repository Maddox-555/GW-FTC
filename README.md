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

## WKU Contributors
* Maddox Simpson
* Chad Wilson
* Nb Bakari
