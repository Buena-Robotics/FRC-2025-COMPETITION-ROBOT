package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Config;

public final class Utils {
    public static Alliance opposing_alliance() {
        return Config.getRobotAlliance() == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }
    public static boolean epsilonOf(final double dist1, final double dist2, final double epsilon){
        return Math.abs(dist1 - dist2) < epsilon;
    }
    public static boolean epsilonOf(final double dist1, final double dist2){
        return epsilonOf(dist1, dist2, 0.02);
    }
}
