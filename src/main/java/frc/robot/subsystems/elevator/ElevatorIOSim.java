package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ElevatorIOSim implements ElevatorIO {
    private final double LIFT_SIM_P = 2;
    private final double LIFT_SIM_D = 0.0;

    private final DCMotor lift_gearbox = DCMotor.getNEO(1);
    private final DCMotorSim lift_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(lift_gearbox, 0.004, Elevator.LIFT_MOTOR_REDUCTION), lift_gearbox);
    private final PIDController lift_controller = new PIDController(LIFT_SIM_P, 0, LIFT_SIM_D);

    private double lift_applied_volts = 0.0;
    private boolean lift_brake_mode = true;

    @Override public void updateInputs(final ElevatorIOInputs inputs) {
        lift_applied_volts = lift_controller.calculate(lift_sim.getAngularPositionRad());
        lift_sim.setInputVoltage(MathUtil.clamp(lift_applied_volts, -12.0, 12.0));
        lift_sim.update(0.02);

        if (lift_sim.getAngularPositionRad() >= Elevator.ELEVATOR_MAX_HEIGHT_INCHES)
            lift_sim.setState(Elevator.ELEVATOR_MAX_HEIGHT_INCHES, 0.0);
        if (lift_sim.getAngularPositionRad() < 0)
            lift_sim.setState(0, 0.0);

        inputs.lift_setpoint_position_inches = lift_controller.getSetpoint();
        inputs.lift_position_inches = lift_sim.getAngularPositionRad();
        inputs.lift_velocity_inches_per_second = lift_sim.getAngularVelocityRadPerSec();
        inputs.lift_applied_volts = lift_sim.getInputVoltage();
        inputs.lift_current_amps = lift_sim.getCurrentDrawAmps();
        inputs.lift_connected = true;
    }

    @Override public void setLiftPosition(double lift_setpoint_position_inches) {
        if (lift_brake_mode)
            lift_controller.setSetpoint(0.0);
        else
            lift_controller.setSetpoint(lift_setpoint_position_inches);
    }

    @Override public void setLiftBrakeMode(final boolean brake_mode) {
        lift_brake_mode = brake_mode;
    }

    @Override public void zeroLiftPosition() {
        lift_sim.setState(0.0, 0.0);
    }
}
