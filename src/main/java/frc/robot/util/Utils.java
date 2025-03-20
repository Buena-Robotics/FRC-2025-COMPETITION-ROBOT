package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Config;

public final class Utils {
    public static Alliance opposingAlliance() {
        return Config.getRobotAlliance() == Alliance.Blue ? Alliance.Red : Alliance.Blue;
    }

    public static boolean epsilonOf(final double dist1, final double dist2, final double epsilon) {
        return Math.abs(dist1 - dist2) < epsilon;
    }

    public static boolean epsilonOf(final double dist1, final double dist2) {
        return epsilonOf(dist1, dist2, 0.02);
    }

    public static Pose2d initialRobotPose() {
        return Config.getRobotAlliance() == Alliance.Blue ? new Pose2d(8.22, 4.78, Rotation2d.fromDegrees(180)) : new Pose2d(9.43, 3.36, Rotation2d.fromDegrees(0));
    }

    public static boolean inBetween(double value, double min, double max){
        return value >= min && value <= max;
    }
}
