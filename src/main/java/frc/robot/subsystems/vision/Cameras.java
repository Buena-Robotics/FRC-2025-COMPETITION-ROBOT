package frc.robot.subsystems.vision;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Cameras {
    // 7 inches behind the centers; 21 inches
    // 10 degrees up

    public static final Camera[] cameras = {
            new Camera(
                "Microsoft_LifeCam_HD-3000",
                new Transform3d(
                    Units.inchesToMeters(-7), 0.0, Units.inchesToMeters(21),
                    new Rotation3d(0.0, Units.degreesToRadians(5), 0.0)),
                3,
                SimCameraProperties.PERFECT_90DEG()),
            new Camera(
                "USB_Camera",
                new Transform3d(
                    Units.inchesToMeters(-12), Units.inchesToMeters(-2), Units.inchesToMeters(21),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(225))),
                10,
                SimCameraProperties.PERFECT_90DEG()),
            new Camera(
                "NEXIGO_HD_Webcam",
                new Transform3d(
                    Units.inchesToMeters(-12), Units.inchesToMeters(2), Units.inchesToMeters(21),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(135))),
                5,
                SimCameraProperties.PERFECT_90DEG()),
    };

    public static record Camera(String name, Transform3d robot_to_camera, double std_dev_factor, SimCameraProperties camera_properties) {

    }
}
