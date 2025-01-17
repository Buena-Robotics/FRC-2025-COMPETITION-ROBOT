package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog public static class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yaw_position = new Rotation2d();
        public double yaw_velocity_radians_per_second = 0.0;
        public double[] odometry_yaw_timestamps = new double[] {};
        public Rotation2d[] odometry_yaw_positions = new Rotation2d[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {}
}
