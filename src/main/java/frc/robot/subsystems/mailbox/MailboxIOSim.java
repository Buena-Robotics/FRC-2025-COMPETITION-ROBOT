package frc.robot.subsystems.mailbox;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MailboxIOSim implements MailboxIO {
    private static final double SIM_SHOOTER_P = 0.001;
    private static final double SIM_SHOOTER_D = 0.0;

    private final DCMotor shooter_gearbox = DCMotor.getNEO(1);
    private final DCMotorSim shooter_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(shooter_gearbox, 0.004, 1.0), shooter_gearbox);
    private final PIDController shooter_controller = new PIDController(SIM_SHOOTER_P, 0, SIM_SHOOTER_D);

    @Override public void updateInputs(final MailboxIOInputs inputs) {
        shooter_sim.update(0.02);

        inputs.shooter_velocity_radians_per_second = shooter_sim.getAngularVelocityRadPerSec();
        inputs.shooter_applied_volts = shooter_sim.getInputVoltage();
        inputs.shooter_current_amps = shooter_sim.getCurrentDrawAmps();
        inputs.shooter_connected = true;
    }

    @Override public void setShooterSpeed(final double shooter_speed) {
        shooter_sim.setInputVoltage(shooter_speed * 12.0);
    }

    @Override public void setShooterVelocity(final double shooter_velocity_radians_per_second) {
        shooter_controller.setSetpoint(shooter_velocity_radians_per_second);
    }
}
