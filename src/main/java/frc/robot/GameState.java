package frc.robot;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.Reef;

public class GameState {
    public static Reef reef = new Reef(Config.getRobotAlliance());
    public static List<Pose2d> corals;
    public static List<Pose2d> algaes;
    public static List<Pose2d> robots; // [Assume]: Robots are 30x30 inches perimeter (not including bumpers)

    private GameState(){}

    public static List<Pair<Translation2d, Translation2d>> getDynamicObstacles(){
        List<Pair<Translation2d, Translation2d>> obstacles = new ArrayList<>();
        return obstacles;
    }
    public static int bestReefTarget(){
        return -1;
    }
    public static int bestCoralStation(){
        return -1;
    }
}
