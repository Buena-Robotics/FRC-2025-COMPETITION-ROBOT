package frc.robot.subsystems.hinge;

public class HingeIOSim implements HingeIO {
    public HingeIOSim() {
    }

    @Override public void updateInputs(final HingeIOInputs inputs) {
        inputs.hinge_position_radians = Math.PI / 2.0;
        inputs.hinge_absolute_position_radians = Math.PI / 2.0;
        inputs.hinge_velocity_radians_per_second = 0.0;
        inputs.hinge_applied_volts = 0.0;
        inputs.hinge_current_amps = 0.0;
        inputs.hinge_connected = true;
    }

    @Override public void setHingeAngle(final double radians) {

    }

    @Override public void setHingeOpenLoop(final double output) {

    }
}
