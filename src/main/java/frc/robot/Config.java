package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

public final class Config {
    public static final RobotMode SIM_MODE = RobotMode.SIM;
    public static final RobotMode ROBOT_MODE = RobotBase.isReal() ? RobotMode.REAL : SIM_MODE;

    public static final RobotType ROBOT_TYPE = RobotType.ROBOT_2025_SCHOOL;

    public static final String DEFAULT_LOG_FOLDER = "/media/sda2/";
    public static final Map<RobotType, String> log_folders_map = Map.of(
        RobotType.ROBOT_2025_COMPETION, DEFAULT_LOG_FOLDER,
        RobotType.ROBOT_2025_PRACTICE, null,
        RobotType.ROBOT_2025_SCHOOL, null,
        RobotType.SETUP_SWERVE, DEFAULT_LOG_FOLDER,
        RobotType.SETUP_TUNING, DEFAULT_LOG_FOLDER);

    public static final Alliance getRobotAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public static enum RobotMode {
        REAL, SIM, REPLAY
    }

    public static enum RobotType {
        ROBOT_2025_COMPETION, ROBOT_2025_PRACTICE, ROBOT_2025_SCHOOL, SETUP_SWERVE, SETUP_TUNING
    }
}
