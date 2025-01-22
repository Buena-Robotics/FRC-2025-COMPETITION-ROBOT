package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog public static class ElevatorIOInputs {
        public boolean lift_connected = true;
        public double lift_setpoint_position_inches = 0.0;
        public double lift_position_inches = 0.0;
        public double lift_velocity_inches_per_second = 0.0;
        public double lift_applied_volts = 0.0;
        public double lift_current_amps = 0.0;

    }

    public default void updateInputs(final ElevatorIOInputs inputs) {}

    public default void setLiftBrakeMode(final boolean brake_mode) {}

    public default void setLiftPosition(double lift_setpoint_position_inches) {}

    public default void zeroLiftPosition() {}
}
