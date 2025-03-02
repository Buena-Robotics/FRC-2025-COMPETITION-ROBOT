package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Cameras.Camera;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhoton implements VisionIO {
    protected final Camera camera_info;
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
    public VisionIOPhoton(final Camera camera_info) {
        this.camera_info = camera_info;
        this.camera = new PhotonCamera(camera_info.name());
        this.robot_to_camera = camera_info.robot_to_camera();
    }

    @Override public void updateInputs(final VisionIOInputs inputs) {
        inputs.camera_connected = camera.isConnected();
        inputs.pipeline_index = camera.getPipelineIndex();
        inputs.driver_mode = camera.getDriverMode();

        if(camera.getCameraMatrix().isEmpty())
            inputs.camera_matrix_opt = camera.getCameraMatrix();
        if(camera.getDistCoeffs().isEmpty())
            inputs.dist_coeffs_opt = camera.getDistCoeffs();
        List<PhotonPipelineResult> unread_results = camera.getAllUnreadResults();
        inputs.photon_results = new PhotonPipelineResult[unread_results.size()];
        for(int i = 0; i < unread_results.size(); i++){
            inputs.photon_results[i] = unread_results.get(i);
        }
    }
}
