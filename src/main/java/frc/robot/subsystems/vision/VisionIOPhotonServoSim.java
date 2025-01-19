package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.Cameras.Camera;

public class VisionIOPhotonServoSim extends VisionIOPhotonServo {
    private static int sim_servo_id = 3;
    private final PhotonCameraSim camera_sim;
    // DCMotor servo_sim = new DCMotor(5.0, 0.1765, 1.25, 0.13,
    // Units.rotationsPerMinuteToRadiansPerSecond(30), 1);

    /**
     * Creates a new VisionIOPhotonVisionSim.
     *
     * @param name
     *            The name of the camera.
     * @param pose_supplier
     *            Supplier for the robot pose to use in simulation.
     */
    public VisionIOPhotonServoSim(final Camera camera_info, final Supplier<Pose2d> robot_pose_supplier, final double servo_radius_cm) {
        super(camera_info, robot_pose_supplier, sim_servo_id++, servo_radius_cm);

        // Initialize vision sim
        if (VisionIOPhotonSim.vision_sim == null) {
            VisionIOPhotonSim.vision_sim = new VisionSystemSim("main");
            VisionIOPhotonSim.vision_sim.addAprilTags(Vision.apriltag_layout);
        }

        // Add sim camera
        final SimCameraProperties camera_properties = SimCameraProperties.LL2_960_720();
        camera_properties.setFPS(30);
        camera_properties.setExposureTimeMs(16);
        camera_sim = new PhotonCameraSim(camera, camera_properties);
        // camera_sim.canSeeTargetPose(null, null)
        VisionIOPhotonSim.vision_sim.addCamera(camera_sim, robot_to_camera);
    }

    @Override public void updateInputs(VisionIOInputs inputs) {
        VisionIOPhotonSim.vision_sim.update(robot_pose_supplier.get());
        VisionIOPhotonSim.vision_sim.adjustCamera(camera_sim, getCameraTransform());

        super.updateInputs(inputs);
    }
}
