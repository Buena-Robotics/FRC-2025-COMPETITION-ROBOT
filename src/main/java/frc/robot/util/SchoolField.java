package frc.robot.util;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class SchoolField extends AprilTagFieldLayout {
    private static List<AprilTag> apriltags = List.of(
        new AprilTag(1, new Pose3d(6.72816, 4.74476, 0.767279, new Rotation3d(0, 0, -2.44346))), // Coral Station
        new AprilTag(2, new Pose3d(1.71345, 3.04525, 1.06994, new Rotation3d(0, 0, 0))), // Processor
        new AprilTag(3, new Pose3d(4.22778, 2.32196, 0.319618, new Rotation3d(0, 0, -Math.PI / 2.0))), // Reef Side
        new AprilTag(4, new Pose3d(7.18936, 3.32708, 1.81997, new Rotation3d(0, 0, Math.PI))), // Barge
        new AprilTag(5, new Pose3d(7.34609, 0.786482, 1.50869, new Rotation3d(0, 0, Math.PI))), // Misc. Cabinet
        new AprilTag(6, new Pose3d(7.94484, 5.42672, 1.40282, new Rotation3d(0, 0, Math.PI)))); // Whiteboard
    private static double field_length = Units.feetToMeters(26.957); // x
    private static double field_width = Units.feetToMeters(27.648); // y

    public SchoolField() {
        super(apriltags, field_length, field_width);
        Pose3d[] tag_poses = new Pose3d[apriltags.size()];
        for (int i = 0; i < apriltags.size(); i++) {
            tag_poses[i] = apriltags.get(i).pose;
        }
        Logger.recordOutput("SchoolField", tag_poses);
    }
}
