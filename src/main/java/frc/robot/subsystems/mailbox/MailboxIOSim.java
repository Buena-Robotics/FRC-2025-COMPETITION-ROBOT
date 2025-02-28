package frc.robot.subsystems.mailbox;

import edu.wpi.first.math.MathUtil;
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

    private double shooter_applied_volts = 0.0;
    private boolean open_loop = false;

    @Override public void updateInputs(final MailboxIOInputs inputs) {
        if (!open_loop) {
            shooter_applied_volts = MathUtil.clamp(shooter_controller.calculate(shooter_sim.getAngularPositionRad()), -12.0, 12.0);
            shooter_sim.setInputVoltage(shooter_applied_volts);
        }
        shooter_sim.update(0.02);

        inputs.shooter_position_radians = shooter_sim.getAngularPositionRad();
        inputs.shooter_velocity_radians_per_second = shooter_sim.getAngularVelocityRadPerSec();
        inputs.shooter_applied_volts = shooter_sim.getInputVoltage();
        inputs.shooter_current_amps = shooter_sim.getCurrentDrawAmps();
        inputs.shooter_connected = true;

        inputs.coral_beambreak_connected = true;
        inputs.coral_beam_broken = false;
    }

    @Override public void resetPosition() {
        shooter_sim.setAngle(0.0);;
    }

    @Override public void setShooterOpenLoop(final double output){
        open_loop = true;
        shooter_applied_volts = output;
        shooter_sim.setInputVoltage(output);
    }

    @Override public void setShooterSpeed(final double shooter_speed) {
        open_loop = false;
        shooter_sim.setInputVoltage(shooter_speed * 12.0);
    }

    @Override public void setShooterPosition(final double shooter_position_radians) {
        open_loop = false;
        shooter_controller.setSetpoint(shooter_position_radians);
    }

    @Override public void setShooterVelocity(final double shooter_velocity_radians_per_second) {
        open_loop = false;
        shooter_controller.setSetpoint(shooter_velocity_radians_per_second);
    }
}
