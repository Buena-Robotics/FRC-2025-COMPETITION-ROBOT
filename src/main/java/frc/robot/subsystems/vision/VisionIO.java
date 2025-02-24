package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public interface VisionIO {
    @AutoLog public static class VisionIOInputs {
        public boolean connected = false;
        public TargetObservation latest_target_observation = new TargetObservation(new Rotation2d(), new Rotation2d());
        public PoseObservation[] pose_observations = new PoseObservation[0];
        public int[] tag_ids = new int[0];
        public Pose3d virtual_cam = new Pose3d();
    }

    /** Represents the angle to a simple target, not used for pose estimation. */
    public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

    /** Represents a robot pose sample used for pose estimation. */
    public static record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tag_count,
        Transform3d robot_to_tag,
        double average_tag_distance,
        PoseObservationType type) {}

    public static enum PoseObservationType {
        MEGATAG_1, MEGATAG_2, PHOTONVISION, VICTINI
    }

    public default void updateInputs(final VisionIOInputs inputs) {}

    public default Pose3d getCameraPose() {
        return new Pose3d();
    }
}
