package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.Cameras.Camera;

import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonSim extends VisionIOPhoton {
    private static VisionSystemSim vision_sim;

    private final Supplier<Pose2d> pose_supplier;
    private final PhotonCameraSim camera_sim;

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name
     *            The name of the camera.
     * @param pose_supplier
     *            Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonSim(final Camera camera_info, final Supplier<Pose2d> pose_supplier) {
        super(camera_info);
        this.pose_supplier = pose_supplier;

        // Initialize vision sim
        if (vision_sim == null) {
            vision_sim = new VisionSystemSim("main");
            vision_sim.addAprilTags(Vision.apriltag_layout);
        }

        // Add sim camera
        final SimCameraProperties camera_properties = SimCameraProperties.LL2_960_720();
        camera_properties.setFPS(30);
        camera_properties.setExposureTimeMs(16);
        camera_sim = new PhotonCameraSim(camera, camera_properties);
        // camera_sim.canSeeTargetPose(null, null)
        vision_sim.addCamera(camera_sim, robot_to_camera);
    }

    @Override public void updateInputs(VisionIOInputs inputs) {
        vision_sim.update(pose_supplier.get());
        super.updateInputs(inputs);
    }
}
