package frc.robot.subsystems.mailbox;

import org.littletonrobotics.junction.AutoLog;

public interface MailboxIO {
    @AutoLog public static class MailboxIOInputs {
        public boolean shooter_connected = true;
        public double shooter_position_radians = 0.0;
        public double shooter_velocity_radians_per_second = 0.0;
        public double shooter_applied_volts = 0.0;
        public double shooter_current_amps = 0.0;

        public boolean coral_beambreak_connected = true;
        public boolean coral_beam_broken = false;
    }

    public default void updateInputs(final MailboxIOInputs inputs) {}

    public default void resetPosition() {}

    public default void setShooterOpenLoop(final double output){}

    public default void setShooterSpeed(final double shooter_speed) {}

    public default void setShooterPosition(final double shooter_position_radians) {}

    public default void setShooterVelocity(final double shooter_velocity_radians_per_second) {}
}
