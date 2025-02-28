package frc.robot.subsystems.vision;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Cameras {
    public static final Camera[] cameras = {
            new Camera(
                "Microsoft_LifeCam_HD-3000",
                new Transform3d(
                    Units.inchesToMeters(-7), 0.0, Units.inchesToMeters(21),
                    new Rotation3d(0.0, Units.degreesToRadians(5), 0.0)),
                0.8,
                SimCameraProperties.LL2_640_480()),
            new Camera(
                "USB_Camera",
                new Transform3d(
                    Units.inchesToMeters(-12), Units.inchesToMeters(-2), Units.inchesToMeters(21),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(225))),
                2.8,
                SimCameraProperties.LL2_960_720()),
            new Camera(
                "NEXIGO_HD_Webcam",
                new Transform3d(
                    Units.inchesToMeters(-12), Units.inchesToMeters(2), Units.inchesToMeters(21),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(135))),
                1.06,
                SimCameraProperties.LL2_960_720()),
    };

    public static record Camera(String name, Transform3d robot_to_camera, double std_dev_factor, SimCameraProperties camera_properties) {

    }
}
