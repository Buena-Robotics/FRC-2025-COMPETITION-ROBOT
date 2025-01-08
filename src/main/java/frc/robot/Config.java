package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;

public final class Config {
    public static final RobotMode SIM_MODE = RobotMode.SIM;
    public static final RobotMode ROBOT_MODE = RobotBase.isReal() ? RobotMode.REAL : SIM_MODE;

    public static final Alliance ROBOT_ALLIANCE = DriverStation.getAlliance().orElse(Alliance.Blue);

    public static enum RobotMode {
        REAL,
        SIM,
        REPLAY
    }
}