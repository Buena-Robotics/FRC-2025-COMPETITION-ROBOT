package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static AprilTagFieldLayout APRILTAG_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
    private static final Transform2d APRILTAG_TO_ROBOT = new Transform2d(Units.inchesToMeters(14), 0, new Rotation2d(Math.PI));

    private static Pose2d apriltagIdToRobotPose(final int id) {
        return APRILTAG_LAYOUT.getTagPose(id).get().toPose2d().plus(APRILTAG_TO_ROBOT);
    }

    public static final int RED_CORAL_STATION_1_TAGID = 1;
    public static final int RED_CORAL_STATION_2_TAGID = 2;
    public static final int RED_REEF_SIDE_1_TAGID = 6;
    public static final int RED_REEF_SIDE_2_TAGID = 7;
    public static final int RED_REEF_SIDE_3_TAGID = 8;
    public static final int RED_REEF_SIDE_4_TAGID = 9;
    public static final int RED_REEF_SIDE_5_TAGID = 10;
    public static final int RED_REEF_SIDE_6_TAGID = 11;
    public static final int RED_PROCESSOR_TAGID = 3;

    public static final int BLUE_CORAL_STATION_1_TAGID = 12;
    public static final int BLUE_CORAL_STATION_2_TAGID = 13;
    public static final int BLUE_REEF_SIDE_1_TAGID = 17;
    public static final int BLUE_REEF_SIDE_2_TAGID = 18;
    public static final int BLUE_REEF_SIDE_3_TAGID = 19;
    public static final int BLUE_REEF_SIDE_4_TAGID = 20;
    public static final int BLUE_REEF_SIDE_5_TAGID = 21;
    public static final int BLUE_REEF_SIDE_6_TAGID = 22;
    public static final int BLUE_PROCESSOR_TAGID = 16;

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
