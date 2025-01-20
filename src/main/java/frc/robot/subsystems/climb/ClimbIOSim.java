package frc.robot.subsystems.climb;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ClimbIOSim implements ClimbIO {
    private final DCMotor winch_gearbox = DCMotor.getNEO(1);
    private final DCMotorSim winch_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(winch_gearbox, 0.004, Climb.WINCH_MOTOR_REDUCTION), winch_gearbox);

    @Override public void updateInputs(final ClimbIOInputs inputs) {
        winch_sim.update(0.02);

        inputs.winch_position_radians = winch_sim.getAngularPositionRad();
        inputs.winch_velocity_radians_per_second = winch_sim.getAngularVelocityRadPerSec();
        inputs.winch_applied_volts = winch_sim.getInputVoltage();
        inputs.winch_current_amps = winch_sim.getCurrentDrawAmps();
        inputs.winch_connected = true;
    }

    @Override public void setWinchSpeed(final double winch_speed) {
        winch_sim.setInputVoltage(winch_speed * 12.0);
    }
}
