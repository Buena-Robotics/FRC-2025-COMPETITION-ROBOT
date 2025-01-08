package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Config;

public final class Utils {
    public static Alliance opposing_alliance() {
        return Config.ROBOT_ALLIANCE == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }
}
