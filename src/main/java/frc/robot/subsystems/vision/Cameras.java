package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Cameras {
    public static final Camera[] cameras = {
            new Camera("camera_0", new Transform3d(0.0, 0.0, Units.inchesToMeters(24), new Rotation3d(0.0, 0.0, 0.0)), 1.0),
            new Camera("camera_1", new Transform3d(0.0, 0.0, Units.inchesToMeters(24), new Rotation3d(0.0, 0.0, Math.PI)), 1.0)
    };

    public static record Camera(String name, Transform3d robot_to_camera, double std_dev_factor) {

    }
}
