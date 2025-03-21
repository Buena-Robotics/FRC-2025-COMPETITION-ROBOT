package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Config.RobotType;
import frc.robot.util.SchoolField;

public class FieldConstants {
    public static AprilTagFieldLayout APRILTAG_LAYOUT = Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? new SchoolField() : AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    public static final Transform2d APRILTAG_TO_ROBOT = new Transform2d(Units.inchesToMeters(13 + 3), 0, new Rotation2d(Math.PI));

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

    private static final double REEF_OFFSET_INCHES = 6.5;
    public static final Transform2d REEF_TRANSFORM_LEFT = new Transform2d(new Translation2d(0.0, Units.inchesToMeters(REEF_OFFSET_INCHES)), new Rotation2d());
    public static final Transform2d REEF_TRANSFORM_RIGHT = new Transform2d(new Translation2d(0.0, Units.inchesToMeters(-REEF_OFFSET_INCHES)), new Rotation2d());

    public static final Transform3d REEF_L2_TRANSFORM_LEFT = new Transform3d(new Translation3d(Units.inchesToMeters(1.625 + 11), Units.inchesToMeters(REEF_OFFSET_INCHES), Units.inchesToMeters(31.875 + 1)), new Rotation3d(0, 0, Math.PI));
    public static final Transform3d REEF_L3_TRANSFORM_LEFT = new Transform3d(new Translation3d(Units.inchesToMeters(1.625 + 11), Units.inchesToMeters(REEF_OFFSET_INCHES), Units.inchesToMeters(47.625 + 1)), new Rotation3d(0, 0, Math.PI));
    public static final Transform3d REEF_L2_TRANSFORM_RIGHT = new Transform3d(new Translation3d(Units.inchesToMeters(1.625 + 11), Units.inchesToMeters(-REEF_OFFSET_INCHES), Units.inchesToMeters(31.875 + 1)), new Rotation3d(0, 0,
        Math.PI));
    public static final Transform3d REEF_L3_TRANSFORM_RIGHT = new Transform3d(new Translation3d(Units.inchesToMeters(1.625 + 11), Units.inchesToMeters(-REEF_OFFSET_INCHES), Units.inchesToMeters(47.625 + 1)), new Rotation3d(0, 0,
        Math.PI));

    public static final Pose2d BLUE_CORAL_STATION_1_POSE = apriltagIdToRobotPose(BLUE_CORAL_STATION_1_TAGID).rotateBy(new Rotation2d(Math.PI));
    public static final Pose2d BLUE_CORAL_STATION_2_POSE = apriltagIdToRobotPose(BLUE_CORAL_STATION_2_TAGID).rotateBy(new Rotation2d(Math.PI));
    public static final Pose2d BLUE_REEF_SIDE_1_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_1_TAGID);
    public static final Pose2d BLUE_REEF_SIDE_2_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_2_TAGID);
    public static final Pose2d BLUE_REEF_SIDE_3_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_3_TAGID);
    public static final Pose2d BLUE_REEF_SIDE_4_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_4_TAGID);
    public static final Pose2d BLUE_REEF_SIDE_5_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_5_TAGID);
    public static final Pose2d BLUE_REEF_SIDE_6_POSE = apriltagIdToRobotPose(BLUE_REEF_SIDE_6_TAGID);
    public static final Pose2d BLUE_PROCESSOR_POSE = apriltagIdToRobotPose(BLUE_PROCESSOR_TAGID);

    public static final Pose2d RED_CORAL_STATION_1_POSE = apriltagIdToRobotPose(RED_CORAL_STATION_1_TAGID).rotateBy(new Rotation2d(Math.PI));
    public static final Pose2d RED_CORAL_STATION_2_POSE = apriltagIdToRobotPose(RED_CORAL_STATION_2_TAGID).rotateBy(new Rotation2d(Math.PI));
    public static final Pose2d RED_REEF_SIDE_1_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_1_TAGID);
    public static final Pose2d RED_REEF_SIDE_2_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_2_TAGID);
    public static final Pose2d RED_REEF_SIDE_3_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_3_TAGID);
    public static final Pose2d RED_REEF_SIDE_4_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_4_TAGID);
    public static final Pose2d RED_REEF_SIDE_5_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_5_TAGID);
    public static final Pose2d RED_REEF_SIDE_6_POSE = apriltagIdToRobotPose(RED_REEF_SIDE_6_TAGID);
    public static final Pose2d RED_PROCESSOR_POSE = apriltagIdToRobotPose(RED_PROCESSOR_TAGID);

    public static final Pose2d[] REEF_SIDE_POSES() {
        if (Config.getRobotAlliance() == Alliance.Blue) {
            return new Pose2d[] {
                    BLUE_REEF_SIDE_1_POSE,
                    BLUE_REEF_SIDE_2_POSE,
                    BLUE_REEF_SIDE_3_POSE,
                    BLUE_REEF_SIDE_4_POSE,
                    BLUE_REEF_SIDE_5_POSE,
                    BLUE_REEF_SIDE_6_POSE,
            };
        }
        return new Pose2d[] {
                RED_REEF_SIDE_1_POSE,
                RED_REEF_SIDE_2_POSE,
                RED_REEF_SIDE_3_POSE,
                RED_REEF_SIDE_4_POSE,
                RED_REEF_SIDE_5_POSE,
                RED_REEF_SIDE_6_POSE,
        };
    }

    public static final Pose3d[] REEF_BRANCHES_POSES() {
        final Pose2d[] reef_side_poses = REEF_SIDE_POSES();
        Pose3d[] poses = new Pose3d[reef_side_poses.length * 4];
        for (int i = 0; i < reef_side_poses.length; i++) {
            poses[(i * 4) + 0] = new Pose3d(reef_side_poses[i]).plus(REEF_L2_TRANSFORM_LEFT);
            poses[(i * 4) + 1] = new Pose3d(reef_side_poses[i]).plus(REEF_L3_TRANSFORM_LEFT);
            poses[(i * 4) + 2] = new Pose3d(reef_side_poses[i]).plus(REEF_L2_TRANSFORM_RIGHT);
            poses[(i * 4) + 3] = new Pose3d(reef_side_poses[i]).plus(REEF_L3_TRANSFORM_RIGHT);
        }

        return poses;
    }

    public static enum ReefBranchHeight {
        L1, L2, L3, L4
    }

    public static enum ReefBranchSide {
        Left, Right
    }
}
