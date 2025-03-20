package frc.robot.subsystems.hinge;

import org.littletonrobotics.junction.AutoLog;

public interface HingeIO {
    @AutoLog public static class HingeIOInputs {
        public boolean hinge_connected = true;
        public double hinge_position_radians = 0.0;
        public double hinge_absolute_position_radians = 0.0;
        public double hinge_velocity_radians_per_second = 0.0;
        public double hinge_applied_volts = 0.0;
        public double hinge_current_amps = 0.0;
    }

    public default void updateInputs(final HingeIOInputs inputs) {}

    public default void setHingeAngle(final double radians) {}

    public default void setHingeOpenLoop(final double output) {}
}
