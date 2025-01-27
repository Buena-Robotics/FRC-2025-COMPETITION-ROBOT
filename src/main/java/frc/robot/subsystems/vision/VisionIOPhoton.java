package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.Cameras.Camera;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhoton implements VisionIO {
    protected final Supplier<Pose2d> robot_pose_supplier;
    protected final Camera camera_info;
    protected final PhotonCamera camera;
    protected final Transform3d robot_to_camera;
    protected Supplier<Transform3d> camera_transform_supplier;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name
     *            The configured name of the camera.
     * @param rotationSupplier
     *            The 3D position of the camera relative to the robot.
     */
    public VisionIOPhoton(final Camera camera_info, final Supplier<Pose2d> robot_pose_supplier, final Supplier<Transform3d> camera_transform_supplier) {
        this.camera_info = camera_info;
        this.camera = new PhotonCamera(camera_info.name());
        this.robot_to_camera = camera_info.robot_to_camera();
        this.robot_pose_supplier = robot_pose_supplier;
        this.camera_transform_supplier = camera_transform_supplier;
    }

    public VisionIOPhoton(final Camera camera_info, final Supplier<Pose2d> robot_pose_supplier) {
        this(camera_info, robot_pose_supplier, () -> new Transform3d());
    }

    public void setCameraTransformSupplier(final Supplier<Transform3d> camera_transform_supplier) {
        this.camera_transform_supplier = camera_transform_supplier;
    }

    @Override public void updateInputs(final VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        // Read new camera observations
        final Set<Short> tag_ids = new HashSet<>();
        final List<PoseObservation> pose_observations = new LinkedList<>();
        for (final PhotonPipelineResult result : camera.getAllUnreadResults()) {
            // Update latest target observation
            if (result.hasTargets()) {
                inputs.latest_target_observation = new TargetObservation(
                    Rotation2d.fromDegrees(result.getBestTarget().getYaw()),
                    Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
            } else {
                inputs.latest_target_observation = new TargetObservation(new Rotation2d(), new Rotation2d());
            }

            // Add pose observation
            if (result.multitagResult.isPresent()) { // Multitag result
                final MultiTargetPNPResult multitag_result = result.multitagResult.get();

                // Calculate robot pose
                final Transform3d field_to_camera = multitag_result.estimatedPose.best;
                final Transform3d field_to_robot = field_to_camera.plus(robot_to_camera.inverse());
                final Pose3d robot_pose = new Pose3d(field_to_robot.getTranslation(), field_to_robot.getRotation());

                // Calculate average tag distance
                double total_tag_distance = 0.0;
                for (PhotonTrackedTarget target : result.targets) {
                    total_tag_distance += target.bestCameraToTarget.getTranslation().getNorm();
                }

                // Add tag IDs
                tag_ids.addAll(multitag_result.fiducialIDsUsed);

                // Add observation
                pose_observations.add(
                    new PoseObservation(
                        result.getTimestampSeconds(), // Timestamp
                        robot_pose, // 3D pose estimate
                        multitag_result.estimatedPose.ambiguity, // Ambiguity
                        multitag_result.fiducialIDsUsed.size(), // Tag count
                        total_tag_distance / result.targets.size(), // Average tag distance
                        PoseObservationType.PHOTONVISION)); // Observation type

            } else if (!result.targets.isEmpty()) { // Single tag result
                final PhotonTrackedTarget target = result.targets.get(0);

                // Calculate robot pose
                final Optional<Pose3d> tag_pose = FieldConstants.APRILTAG_LAYOUT.getTagPose(target.fiducialId);
                if (tag_pose.isPresent()) {
                    final Transform3d field_to_target = new Transform3d(tag_pose.get().getTranslation(), tag_pose.get().getRotation());
                    final Transform3d camera_to_target = target.bestCameraToTarget;
                    final Transform3d field_to_camera = field_to_target.plus(camera_to_target.inverse());
                    final Transform3d field_to_robot = field_to_camera.plus(robot_to_camera.inverse());
                    final Pose3d robot_pose = new Pose3d(field_to_robot.getTranslation(), field_to_robot.getRotation());

                    // Add tag ID
                    tag_ids.add((short) target.fiducialId);

                    // Add observation
                    pose_observations.add(
                        new PoseObservation(
                            result.getTimestampSeconds(), // Timestamp
                            robot_pose, // 3D pose estimate
                            target.poseAmbiguity, // Ambiguity
                            1, // Tag count
                            camera_to_target.getTranslation().getNorm(), // Average tag distance
                            PoseObservationType.PHOTONVISION)); // Observation type
                }
            }
        }

        // Save pose observations to inputs object
        inputs.pose_observations = new PoseObservation[pose_observations.size()];
        for (int i = 0; i < pose_observations.size(); i++) {
            inputs.pose_observations[i] = pose_observations.get(i);
        }

        // Save tag IDs to inputs objects
        inputs.tag_ids = new int[tag_ids.size()];
        int i = 0;
        for (int id : tag_ids) {
            inputs.tag_ids[i++] = id;
        }
    }

    @Override public Pose3d getCameraPose() {
        return new Pose3d(robot_pose_supplier.get()).transformBy(robot_to_camera).transformBy(camera_transform_supplier.get());
    }
}
