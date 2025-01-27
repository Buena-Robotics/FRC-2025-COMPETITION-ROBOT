package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.Cameras.Camera;

import java.util.function.Supplier;

import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonSim extends VisionIOPhoton {
    public static final TargetModel TARGET_MODEL_ALGAE = new TargetModel(Units.inchesToMeters(16.25));
    public static final TargetModel TARGET_MODEL_CORAL = new TargetModel(Units.inchesToMeters(11.875), Units.inchesToMeters(4.5), Units.inchesToMeters(4.5));

    public static VisionSystemSim vision_sim;

    private final PhotonCameraSim camera_sim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name
     *            The name of the camera.
     * @param pose_supplier
     *            Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonSim(final Camera camera_info, final Supplier<Pose2d> robot_pose_supplier) {
        super(camera_info, robot_pose_supplier);

        // Initialize vision sim
        if (vision_sim == null) {
            vision_sim = new VisionSystemSim("main");
            vision_sim.addAprilTags(FieldConstants.APRILTAG_LAYOUT);
        }

        // Add sim camera
        final SimCameraProperties camera_properties = SimCameraProperties.LL2_960_720();
        camera_sim = new PhotonCameraSim(camera, camera_properties);
        camera_sim.enableDrawWireframe(true);
        vision_sim.addCamera(camera_sim, robot_to_camera);
    }

    @Override public void updateInputs(VisionIOInputs inputs) {
        vision_sim.update(robot_pose_supplier.get());
        super.updateInputs(inputs);
        // final List<Pose3d> algae_poses =
        // SimulatedArena.getInstance().getGamePiecesByType("Algae");
        // final List<Pose3d> coral_poses =
        // SimulatedArena.getInstance().getGamePiecesByType("Coral");
        // for (int i = 0; i < algae_poses.size(); i++) {
        // vision_sim.addVisionTargets("algae", new VisionTargetSim[] { new
        // VisionTargetSim(algae_poses.get(i), TARGET_MODEL_ALGAE) });
        // }
        // for (int i = 0; i < coral_poses.size(); i++) {
        // vision_sim.addVisionTargets("coral", new VisionTargetSim[] { new
        // VisionTargetSim(coral_poses.get(i), TARGET_MODEL_CORAL) });
        // }
    }
}
