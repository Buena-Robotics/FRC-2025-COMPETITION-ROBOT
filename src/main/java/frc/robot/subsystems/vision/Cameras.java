package frc.robot.subsystems.vision;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class Cameras {
    public static final Camera[] cameras = {
            new Camera(
                "Microsoft_LifeCam_HD-3000", // ORANGE PI
                new Transform3d(
                    Units.inchesToMeters(-13 + 1.75), Units.inchesToMeters(1.0 / 8.0), Units.inchesToMeters(21.625),
                    new Rotation3d(0.0, 0.0, Units.degreesToRadians(180-.5))),
                0.8,
                SimCameraProperties.LL2_640_480()),
            new Camera(
                "C270_HD_WEBCAM",
                new Transform3d(
                    Units.inchesToMeters(-11.5), Units.inchesToMeters(3), Units.inchesToMeters(21.5),
                    new Rotation3d(0.0, Units.degreesToRadians(1), Units.degreesToRadians(133))),
                0.45,
                SimCameraProperties.LL2_640_480()),
            new Camera(
                "USB_Camera",
                new Transform3d(
                    Units.inchesToMeters(-5), Units.inchesToMeters(-(7.0/16.0)), Units.inchesToMeters(20),
                    new Rotation3d(0.0, Units.degreesToRadians(-2), Units.degreesToRadians(-1))),
                    2.0,
                SimCameraProperties.LL2_960_720()),
            new Camera(
                "NEXIGO_HD_Webcam",
                new Transform3d(
                    Units.inchesToMeters(-11.25), Units.inchesToMeters(-2 - (1.0/8.0)), Units.inchesToMeters(22),
                    new Rotation3d(0.0,Units.degreesToRadians(0.5), Units.degreesToRadians(238.5))),
                0.5,
                SimCameraProperties.LL2_960_720()),
    };

    public static record Camera(String name, Transform3d robot_to_camera, double std_dev_factor, SimCameraProperties camera_properties) {

    }
}
