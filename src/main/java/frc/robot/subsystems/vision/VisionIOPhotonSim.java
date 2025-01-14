package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
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
    public VisionIOPhotonSim(
            String name, Transform3d robot_to_camera, Supplier<Pose2d> pose_supplier) {
        super(name, robot_to_camera);
        this.pose_supplier = pose_supplier;

        // Initialize vision sim
        if (vision_sim == null) {
            vision_sim = new VisionSystemSim("main");
            vision_sim.addAprilTags(Vision.apriltag_layout);
        }

        // Add sim camera
        var camera_properties = new SimCameraProperties();
        camera_sim = new PhotonCameraSim(camera, camera_properties);
        vision_sim.addCamera(camera_sim, robot_to_camera);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        vision_sim.update(pose_supplier.get());
        super.updateInputs(inputs);
    }
}