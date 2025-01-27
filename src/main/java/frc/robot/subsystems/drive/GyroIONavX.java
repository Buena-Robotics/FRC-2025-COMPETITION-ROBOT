package frc.robot.subsystems.drive;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

public class GyroIONavX implements GyroIO {
    private final AHRS navx = new AHRS(NavXComType.kUSB1, (byte) Drive.ODOMETRY_FREQUENCY_HERTZ);
    private final Queue<Double> yaw_position_queue;
    private final Queue<Double> yaw_timestamp_queue;

    public GyroIONavX() {
        yaw_timestamp_queue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yaw_position_queue = SparkOdometryThread.getInstance().registerSignal(navx::getAngle);
    }

    @Override public void updateInputs(final GyroIOInputs inputs) {
        inputs.connected = navx.isConnected();
        inputs.yaw_position = Rotation2d.fromDegrees(-navx.getAngle());
        inputs.yaw_velocity_radians_per_second = Units.degreesToRadians(-navx.getRawGyroZ());

        inputs.odometry_yaw_timestamps = yaw_timestamp_queue.stream()
            .mapToDouble((Double value) -> value)
            .toArray();
        inputs.odometry_yaw_positions = yaw_position_queue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
        yaw_timestamp_queue.clear();
        yaw_position_queue.clear();
    }
}
