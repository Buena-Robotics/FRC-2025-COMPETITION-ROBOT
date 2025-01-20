package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
    @AutoLog public static class ClimbIOInputs {
        public boolean winch_connected = true;
        public double winch_position_radians = 0.0;
        public double winch_velocity_radians_per_second = 0.0;
        public double winch_applied_volts = 0.0;
        public double winch_current_amps = 0.0;
    }

    public default void updateInputs(final ClimbIOInputs inputs) {}

    public default void setWinchSpeed(final double winch_speed) {}
}
