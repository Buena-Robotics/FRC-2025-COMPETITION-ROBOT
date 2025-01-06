package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public final class Config {
    public final static RobotMode SIM_MODE = RobotMode.SIM;
    public final static RobotMode ROBOT_MODE = RobotBase.isReal() ? RobotMode.REAL : SIM_MODE;
    
    public final static Alliance ROBOT_ALLIANCE = DriverStation.getAlliance().orElse(Alliance.Blue);

    public static enum RobotMode {
        REAL, SIM, REPLAY
    }
}
