package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Config;
import frc.robot.Config.RobotMode;

public class Vision extends SubsystemBase {
    CameraIO[] cameras = Config.ROBOT_MODE == RobotMode.REAL
            ? new CameraIO[] {new CameraPhoton()}
            : new CameraIO[] {new CameraPhotonSim()};
}
