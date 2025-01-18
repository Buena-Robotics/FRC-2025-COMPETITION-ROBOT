package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
    @AutoLog public static class ModuleIOInputs {
        public boolean drive_connected = false;
        public double drive_position_radians = 0.0;
        public double drive_velocity_radians_per_second = 0.0;
        public double drive_applied_volts = 0.0;
        public double drive_current_amps = 0.0;

        public boolean turn_connected = false;
        public Rotation2d turn_position = new Rotation2d();
        public Rotation2d turn_absolute_position = new Rotation2d();
        public double turn_velocity_radians_per_second = 0.0;
        public double turn_applied_volts = 0.0;
        public double turn_current_amps = 0.0;

        public double[] odometry_timestamps = new double[] {};
        public double[] odometry_drive_position_radians = new double[] {};
        public Rotation2d[] odometry_turn_positions = new Rotation2d[] {};
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(final ModuleIOInputs inputs) {}

    public default void setDriveOpenLoop(final double output) {}
    public default void setDriveVelocity(final double velocity_radians_per_second) {}
    public default void setDriveBrakeMode(final boolean brake_mode) {}

    public default void setTurnOpenLoop(final double output) {}
    public default void setTurnPosition(final Rotation2d rotation) {}
    public default void setTurnBrakeMode(final boolean brake_mode) {}
}
