package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/** IO implementation for real VictiniVision Jetson Nano Hardware. */
public class VisionIOVictini implements VisionIO {
    @SuppressWarnings("unused") private final NetworkTable vi_table = NetworkTableInstance.getDefault().getTable("victini");

    @Override public void updateInputs(final VisionIOInputs inputs) {
        inputs.connected = true;
        inputs.pose_observations = new PoseObservation[]{
            new PoseObservation(0, new Pose3d(), 0, 0, 0, PoseObservationType.VICTINI
        )};
    }
}
