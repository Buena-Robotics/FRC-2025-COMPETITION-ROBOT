package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.SharedPhotonPoseEstimator.EstimatedRobotPose;
import frc.robot.subsystems.vision.SharedPhotonPoseEstimator.PoseStrategy;
import frc.robot.subsystems.vision.VisionIO.VisionIOInputs;

import static edu.wpi.first.units.Units.*;

import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class Vision extends SubsystemBase {
    // Basic filtering thresholds
    public static double max_ambiguity = 0.2;
    public static double max_z_error = 0.25;
    public static double max_pitch_roll_error_radians = 0.2;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linear_std_dev_baseline_meters = 0.25; // Meters
    public static double angular_std_dev_baseline_radians = Math.PI / 6.0; // Radians

    // Multipliers to apply for MegaTag 2 observations
    public static double linear_std_dev_megatag_2_factor = 0.5; // More stable than full 3D solve
    public static double angular_std_dev_megatag_2_factor = Double.POSITIVE_INFINITY; // No rotation data available

    private final Distance SINGLE_TO_MULTI_TAG_POSE_DELTA = Meters.of(0.5);
    private final Distance MAX_TAG_DISTANCE = Meters.of(5.0);

    private final Drive drive;
    private final VisionConsumer consumer;
    private final VisionIO[] io;
    private final VisionIOInputs[] inputs;
    private final SharedPhotonPoseEstimator[] estimators;
    private final Alert[] disconnected_alerts;
    private final BooleanSupplier force_single_tag;

    public Vision(VisionConsumer consumer, Drive drive, BooleanSupplier force_single_tag, VisionIO... io) {
        this.consumer = consumer;
        this.drive = drive;
        this.io = io;
        this.force_single_tag = force_single_tag;

        // Initialize inputs
        this.inputs = new VisionIOInputs[io.length];
        this.estimators = new SharedPhotonPoseEstimator[io.length];
        this.disconnected_alerts = new Alert[io.length];
        for (int i = 0; i < inputs.length; i++) {
            this.inputs[i] = new VisionIOInputs();
            this.estimators[i] = new SharedPhotonPoseEstimator(
                FieldConstants.APRILTAG_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, Cameras.cameras[i].robot_to_camera());
            this.estimators[i].setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
            this.disconnected_alerts[i] = new Alert(
                "Vision camera " + Integer.toString(i) + " is disconnected.", AlertType.kWarning);
        }
    }

    private Optional<EstimatedRobotPose> singleTagEstimate(final SharedPhotonPoseEstimator estimator, final Optional<EstimatedRobotPose> opt_estimate_pose) {
        if (opt_estimate_pose.isEmpty())
            return Optional.empty();
        // Make sure the measurement is valid
        EstimatedRobotPose estimate_pose = opt_estimate_pose.get();

        // Get distance to closest tag
        var closestTagDistance = Meters.of(100.0);
        // Loop through all targets used for this estimate
        for (var target : estimate_pose.targetsUsed) {
            // Get tag
            var tag = FieldConstants.APRILTAG_LAYOUT.getTagPose(target.getFiducialId());
            // Get distance to tag
            var tagDistance = Meters.of(target.getBestCameraToTarget().getTranslation().getNorm());
            // Get pose estimate based on just this tag
            var singleTargetPose = tag.get()
                .transformBy(target.getBestCameraToTarget().inverse())
                .transformBy(estimator.getRobotToCameraTransform().inverse());
            // Ignore if single tag pose estimate is too far from multi-tag estimate
            if (estimate_pose.estimatedPose.relativeTo(singleTargetPose).getTranslation().getNorm() > SINGLE_TO_MULTI_TAG_POSE_DELTA.in(Meters))
                return Optional.empty();
            // Check if tag distance is closest yet
            if (tagDistance.lte(closestTagDistance))
                closestTagDistance = tagDistance;
        }

        // Ignore if tags are too far
        if (closestTagDistance.gte(MAX_TAG_DISTANCE))
            return Optional.empty();

        return opt_estimate_pose;
    }

    private Optional<EstimatedRobotPose> estimate(final SharedPhotonPoseEstimator estimator, final PhotonPipelineResult result) {
        if (!result.hasTargets())
            return Optional.empty();
        if (result.multitagResult.isPresent())
            return estimator.update(result, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        if (!DriverStation.isEnabled() || Math.abs(drive.yawRate()) >= 0.04 || force_single_tag.getAsBoolean()) {
            return singleTagEstimate(estimator, estimator.update(result, PoseStrategy.AVERAGE_BEST_TARGETS));
        }
        return estimator.update(result, PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
    }

    @Override public void periodic() {
        SharedPhotonPoseEstimator.addHeadingData(Timer.getTimestamp(), drive.getPose().getRotation());
        List<EstimatedRobotPose> estimated_robot_poses = new LinkedList<>();
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs(cameraToKey(i), inputs[i]);
            for (PhotonPipelineResult photon_result : inputs[i].photon_results) {
                Optional<EstimatedRobotPose> potential_estimate = estimate(estimators[i], photon_result);
                if (potential_estimate.isPresent())
                    estimated_robot_poses.add(potential_estimate.get());
            }
        }

        // Initialize logging values
        List<Pose3d> all_tag_poses = new LinkedList<>();
        List<Pose3d> all_robot_poses = new LinkedList<>();
        List<Pose3d> all_robot_poses_accepted = new LinkedList<>();
        List<Pose3d> all_robot_poses_rejected = new LinkedList<>();

        // Loop over cameras
        for (int i = 0; i < io.length; i++) {
            // Update disconnected alert
            disconnected_alerts[i].set(!inputs[i].camera_connected);

            // Initialize logging values
            List<Pose3d> tag_poses = new LinkedList<>();
            List<Pose3d> robot_poses = new LinkedList<>();
            List<Pose3d> robot_poses_accepted = new LinkedList<>();
            List<Pose3d> robot_poses_rejected = new LinkedList<>();

            // Add tag poses
            for (PhotonPipelineResult photon_result : inputs[i].photon_results) {
                tag_poses.addAll(
                    photon_result
                        .getTargets()
                        .stream()
                        .map((PhotonTrackedTarget target) -> {
                            return FieldConstants.APRILTAG_LAYOUT.getTagPose(target.fiducialId).orElseGet(() -> new Pose3d());
                        }).toList());
            }

            // Loop over pose observations
            for (EstimatedRobotPose observation : estimated_robot_poses) {
                // Check whether to reject pose
                final int tag_count = observation.targetsUsed.size();
                boolean reject_pose = tag_count == 0 // Must have at least one tag
                    || (tag_count == 1 && observation.ambiguity > max_ambiguity) // Cannot be high ambiguity
                    || Math.abs(observation.estimatedPose.getZ()) > max_z_error // Must have realistic Z coordinate

                    // Must be within the field boundaries
                    || observation.estimatedPose.getX() < 0.0 || observation.estimatedPose.getX() > FieldConstants.APRILTAG_LAYOUT.getFieldLength() || observation.estimatedPose.getY() < 0.0 || observation.estimatedPose
                        .getY() > FieldConstants.APRILTAG_LAYOUT
                            .getFieldWidth()

                    || Math.abs(observation.estimatedPose.getRotation().getX()) > max_pitch_roll_error_radians || Math.abs(observation.estimatedPose.getRotation().getY()) > max_pitch_roll_error_radians;

                // Add pose to log
                robot_poses.add(observation.estimatedPose);
                if (reject_pose) {
                    robot_poses_rejected.add(observation.estimatedPose);
                } else {
                    robot_poses_accepted.add(observation.estimatedPose);
                }

                // Skip if rejected
                if (reject_pose) {
                    continue;
                }

                var average_tag_distance = observation.targetsUsed
                    .stream()
                    .map((PhotonTrackedTarget target) -> target.bestCameraToTarget.getTranslation().getNorm())
                    .reduce(0.0, Double::sum) / tag_count;

                // Calculate standard deviations
                double std_dev_factor = Math.pow(average_tag_distance, 2.0) / tag_count;
                double linear_std_dev = linear_std_dev_baseline_meters * std_dev_factor;
                double angular_std_dev = angular_std_dev_baseline_radians * std_dev_factor * (DriverStation.isEnabled() ? 10 : 1);

                if (observation.strategy == PoseStrategy.PNP_DISTANCE_TRIG_SOLVE || observation.strategy == PoseStrategy.CONSTRAINED_SOLVEPNP) {
                    linear_std_dev *= linear_std_dev_megatag_2_factor;
                    angular_std_dev *= angular_std_dev_megatag_2_factor;
                }
                if (i < Cameras.cameras.length) {
                    linear_std_dev *= Cameras.cameras[i].std_dev_factor();
                    angular_std_dev *= Cameras.cameras[i].std_dev_factor();
                }

                // Send vision observation
                consumer.accept(
                    observation.estimatedPose.toPose2d(),
                    observation.timestampSeconds,
                    VecBuilder.fill(linear_std_dev, linear_std_dev, angular_std_dev));
            }

            // Log camera datadata
            String camera_key = cameraToKey(i);
            Logger.recordOutput(camera_key + "/VirtualCameraPose", getCameraWorldPose(i));
            Logger.recordOutput(camera_key + "/TagPoses", tag_poses.toArray(new Pose3d[tag_poses.size()]));
            Logger.recordOutput(camera_key + "/RobotPoses", robot_poses.toArray(new Pose3d[robot_poses.size()]));
            Logger.recordOutput(camera_key + "/RobotPosesAccepted", robot_poses_accepted.toArray(new Pose3d[robot_poses_accepted.size()]));
            Logger.recordOutput(camera_key + "/RobotPosesRejected", robot_poses_rejected.toArray(new Pose3d[robot_poses_rejected.size()]));
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

    private String cameraToKey(int camera_index) {
        return "Vision/Camera" + Integer.toString(camera_index);
    }

    public Pose3d getCameraWorldPose(int camera_index) {
        return new Pose3d(drive.getPose()).transformBy(Cameras.cameras[camera_index].robot_to_camera());
    }

    @FunctionalInterface public static interface VisionConsumer {
        public void accept(
            Pose2d vision_robot_pose_meters,
            double timestamp_seconds,
            Matrix<N3, N1> vision_measurements_std_devs);
    }
}
