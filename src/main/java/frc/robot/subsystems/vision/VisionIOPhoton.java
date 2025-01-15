package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhoton implements VisionIO {
    protected final PhotonCamera camera;
    protected final Transform3d robot_to_camera;

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name
     *            The configured name of the camera.
     * @param rotationSupplier
     *            The 3D position of the camera relative to the robot.
     */
    public VisionIOPhoton(String name, Transform3d robot_to_camera) {
        camera = new PhotonCamera(name);
        this.robot_to_camera = robot_to_camera;
    }

    @Override public void updateInputs(VisionIOInputs inputs) {
        inputs.connected = camera.isConnected();

        // Read new camera observations
        Set<Short> tag_ids = new HashSet<>();
        List<PoseObservation> pose_observations = new LinkedList<>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
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
                MultiTargetPNPResult multitag_result = result.multitagResult.get();

                // Calculate robot pose
                Transform3d fieldToCamera = multitag_result.estimatedPose.best;
                Transform3d fieldToRobot = fieldToCamera.plus(robot_to_camera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

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
                        robotPose, // 3D pose estimate
                        multitag_result.estimatedPose.ambiguity, // Ambiguity
                        multitag_result.fiducialIDsUsed.size(), // Tag count
                        total_tag_distance / result.targets.size(), // Average tag distance
                        PoseObservationType.PHOTONVISION)); // Observation type

            } else if (!result.targets.isEmpty()) { // Single tag result
                PhotonTrackedTarget target = result.targets.get(0);

                // Calculate robot pose
                Optional<Pose3d> tag_pose = Vision.apriltag_layout.getTagPose(target.fiducialId);
                if (tag_pose.isPresent()) {
                    Transform3d field_to_target = new Transform3d(tag_pose.get().getTranslation(), tag_pose.get().getRotation());
                    Transform3d camera_to_target = target.bestCameraToTarget;
                    Transform3d field_to_camera = field_to_target.plus(camera_to_target.inverse());
                    Transform3d field_to_robot = field_to_camera.plus(robot_to_camera.inverse());
                    Pose3d robot_pose = new Pose3d(field_to_robot.getTranslation(), field_to_robot.getRotation());

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
}
