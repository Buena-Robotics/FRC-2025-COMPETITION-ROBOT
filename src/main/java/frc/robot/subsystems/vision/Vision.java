package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.vision.VisionIO.PoseObservation;
import frc.robot.subsystems.vision.VisionIO.PoseObservationType;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

public class Vision extends SubsystemBase {
    // AprilTag layout
    public static AprilTagFieldLayout apriltag_layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Basic filtering thresholds
    public static double max_ambiguity = 0.3;
    public static double max_z_error = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linear_std_dev_baseline_meters = 0.02; // Meters
    public static double angular_std_dev_baseline_radians = 0.06; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static double linear_std_dev_megatag_2_factor = 0.5; // More stable than full 3D solve
    public static double angular_std_dev_megatag_2_factor = Double.POSITIVE_INFINITY; // No rotation data available

    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputsAutoLogged[] inputs;
    private final Alert[] disconnected_alerts;

    public Vision(VisionConsumer consumer, VisionIO... io) {
        this.consumer = consumer;
        this.io = io;

        // Initialize inputs
        this.inputs = new VisionIOInputsAutoLogged[io.length];
        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = new VisionIOInputsAutoLogged();
        }

        // Initialize disconnected alerts
        this.disconnected_alerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            disconnected_alerts[i] = new Alert(
                "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    /**
     * Returns the X angle to the best target, which can be used for simple servoing
     * with vision.
     *
     * @param camera_index
     *            The index of the camera to use.
     */
    public Rotation2d getTargetX(int camera_index) {
        return inputs[camera_index].latest_target_observation.tx();
    }

    @Override public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
        }

        // Initialize logging values
        List<Pose3d> all_tag_poses = new LinkedList<>();
        List<Pose3d> all_robot_poses = new LinkedList<>();
        List<Pose3d> all_robot_poses_accepted = new LinkedList<>();
        List<Pose3d> all_robot_poses_rejected = new LinkedList<>();

        // Loop over cameras
        for (int i = 0; i < io.length; i++) {
            // Update disconnected alert
            disconnected_alerts[i].set(!inputs[i].connected);

            // Initialize logging values
            List<Pose3d> tag_poses = new LinkedList<>();
            List<Pose3d> robot_poses = new LinkedList<>();
            List<Pose3d> robot_poses_accepted = new LinkedList<>();
            List<Pose3d> robot_poses_rejected = new LinkedList<>();

            // Add tag poses
            for (int tag_id : inputs[i].tag_ids) {
                Optional<Pose3d> tag_pose = apriltag_layout.getTagPose(tag_id);
                if (tag_pose.isPresent()) {
                    tag_poses.add(tag_pose.get());
                }
            }

            // Loop over pose observations
            for (PoseObservation observation : inputs[i].pose_observations) {
                // Check whether to reject pose
                boolean reject_pose = observation.tag_count() == 0 // Must have at least one tag
                    || (observation.tag_count() == 1 && observation.ambiguity() > max_ambiguity) // Cannot be high ambiguity
                    || Math.abs(observation.pose().getZ()) > max_z_error // Must have realistic Z coordinate

                    // Must be within the field boundaries
                    || observation.pose().getX() < 0.0 || observation.pose().getX() > apriltag_layout.getFieldLength() || observation.pose().getY() < 0.0 || observation.pose().getY() > apriltag_layout.getFieldWidth();

                // Add pose to log
                robot_poses.add(observation.pose());
                if (reject_pose) {
                    robot_poses_rejected.add(observation.pose());
                } else {
                    robot_poses_accepted.add(observation.pose());
                }

                // Skip if rejected
                if (reject_pose) {
                    continue;
                }

                // Calculate standard deviations
                double std_dev_factor = Math.pow(observation.average_tag_distance(), 2.0) / observation.tag_count();
                double linear_std_dev = linear_std_dev_baseline_meters * std_dev_factor;
                double angular_std_dev = angular_std_dev_baseline_radians * std_dev_factor;
                if (observation.type() == PoseObservationType.MEGATAG_2) {
                    linear_std_dev *= linear_std_dev_megatag_2_factor;
                    angular_std_dev *= angular_std_dev_megatag_2_factor;
                }
                if (i < Cameras.cameras.length) {
                    linear_std_dev *= Cameras.cameras[i].std_dev_factor();
                    angular_std_dev *= Cameras.cameras[i].std_dev_factor();
                }

                // Send vision observation
                consumer.accept(
                    observation.pose().toPose2d(),
                    observation.timestamp(),
                    VecBuilder.fill(linear_std_dev, linear_std_dev, angular_std_dev));
            }

            // Log camera datadata
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(i) + "/TagPoses",
                tag_poses.toArray(new Pose3d[tag_poses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(i) + "/RobotPoses",
                robot_poses.toArray(new Pose3d[robot_poses.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(i) + "/RobotPosesAccepted",
                robot_poses_accepted.toArray(new Pose3d[robot_poses_accepted.size()]));
            Logger.recordOutput(
                "Vision/Camera" + Integer.toString(i) + "/RobotPosesRejected",
                robot_poses_rejected.toArray(new Pose3d[robot_poses_rejected.size()]));
            all_tag_poses.addAll(tag_poses);
            all_robot_poses.addAll(robot_poses);
            all_robot_poses_accepted.addAll(robot_poses_accepted);
            all_robot_poses_rejected.addAll(robot_poses_rejected);
        }

        // Log summary data
        Logger.recordOutput(
            "Vision/Summary/TagPoses", all_tag_poses.toArray(new Pose3d[all_tag_poses.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPoses", all_robot_poses.toArray(new Pose3d[all_robot_poses.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesAccepted",
            all_robot_poses_accepted.toArray(new Pose3d[all_robot_poses_accepted.size()]));
        Logger.recordOutput(
            "Vision/Summary/RobotPosesRejected",
            all_robot_poses_rejected.toArray(new Pose3d[all_robot_poses_rejected.size()]));
    }

    @FunctionalInterface public static interface VisionConsumer {
        public void accept(
            Pose2d vision_robot_pose_meters,
            double timestamp_seconds,
            Matrix<N3, N1> vision_measurements_std_devs);
    }
}
