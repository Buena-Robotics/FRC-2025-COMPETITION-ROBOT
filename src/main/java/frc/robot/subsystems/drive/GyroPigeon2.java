package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import java.util.Queue;

public class GyroPigeon2 implements GyroIO {
    private final int PIGEON_CAN_ID = 9;

    private final Pigeon2 pigeon = new Pigeon2(PIGEON_CAN_ID);
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final Queue<Double> yaw_position_queue;
    private final Queue<Double> yaw_timestamp_queue;
    private final StatusSignal<AngularVelocity> yaw_velocity = pigeon.getAngularVelocityZWorld();

    public GyroPigeon2() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY_HERTZ);
        yaw_velocity.setUpdateFrequency(50.0);
        pigeon.optimizeBusUtilization();
        yaw_timestamp_queue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yaw_position_queue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
    }

    @Override public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yaw_velocity).equals(StatusCode.OK);
        inputs.yaw_position = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yaw_velocity_radians_per_second = Units.degreesToRadians(yaw_velocity.getValueAsDouble());

        inputs.odometry_yaw_timestamps = yaw_timestamp_queue.stream()
            .mapToDouble((Double value) -> value)
            .toArray();
        inputs.odometry_yaw_positions = yaw_position_queue.stream().map(Rotation2d::fromDegrees).toArray(Rotation2d[]::new);
        yaw_timestamp_queue.clear();
        yaw_position_queue.clear();
    }
}
