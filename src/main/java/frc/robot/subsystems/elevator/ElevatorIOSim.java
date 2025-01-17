package frc.robot.subsystems.elevator;

public class ElevatorIOSim implements ElevatorIO {
    @Override public void updateInputs(final ElevatorIOInputs inputs) {
        inputs.lift_connected = true;
        inputs.lift_position_inches = 0.0;
        inputs.lift_velocity_inches_per_second = 0.0;
        inputs.lift_applied_volts = 0.0;
        inputs.lift_current_amps = 0.0;
    }
    @Override public void setLiftPosition(final double lift_setpoint_position_inches){

    }
}
