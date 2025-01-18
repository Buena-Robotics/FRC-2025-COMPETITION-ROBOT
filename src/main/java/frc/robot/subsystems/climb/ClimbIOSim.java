package frc.robot.subsystems.climb;

public class ClimbIOSim implements ClimbIO {
    public ClimbIOSim() {

    }

    @Override public void updateInputs(final ClimbIOInputs inputs) {
        inputs.winch_connected = true;
        inputs.winch_velocity_radians_per_second = 0.0;
        inputs.winch_applied_volts = 0.0;
        inputs.winch_current_amps = 0.0;
    }

    @Override public void setWinchSpeed(final double winch_speed) {

    }
}
