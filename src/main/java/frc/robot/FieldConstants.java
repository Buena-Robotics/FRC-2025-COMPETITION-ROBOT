package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Config.RobotType;
import frc.robot.util.SchoolField;

public class FieldConstants {
    public static AprilTagFieldLayout APRILTAG_LAYOUT = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? new SchoolField() : AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private static final Transform2d APRILTAG_TO_ROBOT = new Transform2d(Units.inchesToMeters(14), 0, new Rotation2d(Math.PI));

    private static Pose2d apriltagIdToRobotPose(final int id) {
        return APRILTAG_LAYOUT.getTagPose(id).get().toPose2d().plus(APRILTAG_TO_ROBOT);
    }

    public static final int RED_CORAL_STATION_1_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 1 : 1;
    public static final int RED_CORAL_STATION_2_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 1 : 2;
    public static final int RED_REEF_SIDE_1_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 6;
    public static final int RED_REEF_SIDE_2_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 7;
    public static final int RED_REEF_SIDE_3_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 8;
    public static final int RED_REEF_SIDE_4_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 9;
    public static final int RED_REEF_SIDE_5_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 10;
    public static final int RED_REEF_SIDE_6_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 11;
    public static final int RED_PROCESSOR_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 2 : 3;

    public static final int BLUE_CORAL_STATION_1_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 1 : 12;
    public static final int BLUE_CORAL_STATION_2_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 1 : 13;
    public static final int BLUE_REEF_SIDE_1_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 17;
    public static final int BLUE_REEF_SIDE_2_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 18;
    public static final int BLUE_REEF_SIDE_3_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 19;
    public static final int BLUE_REEF_SIDE_4_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 20;
    public static final int BLUE_REEF_SIDE_5_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 21;
    public static final int BLUE_REEF_SIDE_6_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 3 : 22;
    public static final int BLUE_PROCESSOR_TAGID = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? 2 : 16;

    public static final Pose2d BLUE_CORAL_STATION_1_POSE = apriltagIdToRobotPose(BLUE_CORAL_STATION_1_TAGID);
    public static final Pose2d BLUE_CORAL_STATION_2_POSE = apriltagIdToRobotPose(BLUE_CORAL_STATION_2_TAGID);
    public static final Pose2d BLUE_REEF_SIDE_1_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_1_TAGID);
    public static final Pose2d BLUE_REEF_SIDE_2_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_2_TAGID);
    public static final Pose2d BLUE_REEF_SIDE_3_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_3_TAGID);
    public static final Pose2d BLUE_REEF_SIDE_4_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_4_TAGID);
    public static final Pose2d BLUE_REEF_SIDE_5_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_5_TAGID);
    public static final Pose2d BLUE_REEF_SIDE_6_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_6_TAGID);
    public static final Pose2d BLUE_PROCESSOR_POSE = apriltagIdToRobotPose(BLUE_PROCESSOR_TAGID);

    public static final Pose2d RED_CORAL_STATION_1_POSE = apriltagIdToRobotPose(RED_CORAL_STATION_1_TAGID);
    public static final Pose2d RED_CORAL_STATION_2_POSE = apriltagIdToRobotPose(RED_CORAL_STATION_2_TAGID);
    public static final Pose2d RED_REEF_SIDE_1_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_1_TAGID);
    public static final Pose2d RED_REEF_SIDE_2_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_2_TAGID);
    public static final Pose2d RED_REEF_SIDE_3_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_3_TAGID);
    public static final Pose2d RED_REEF_SIDE_4_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_4_TAGID);
    public static final Pose2d RED_REEF_SIDE_5_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_5_TAGID);
    public static final Pose2d RED_REEF_SIDE_6_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_6_TAGID);
    public static final Pose2d RED_PROCESSOR_POSE = apriltagIdToRobotPose(RED_PROCESSOR_TAGID);
}
