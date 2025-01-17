package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.util.Units;
import frc.robot.util.SparkUtil;
import org.ironmaple.simulation.drivesims.GyroSimulation;

public class GyroSim implements GyroIO {
    private final GyroSimulation gyroSimulation;

    public GyroSim(GyroSimulation gyroSimulation) {
        this.gyroSimulation = gyroSimulation;
    }

    @Override public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yaw_position = gyroSimulation.getGyroReading();
        inputs.yaw_velocity_radians_per_second = Units.degreesToRadians(
            gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));
        inputs.odometry_yaw_timestamps = SparkUtil.getSimulationOdometryTimeStamps();
        inputs.odometry_yaw_positions = gyroSimulation.getCachedGyroReadings();
    }
}
