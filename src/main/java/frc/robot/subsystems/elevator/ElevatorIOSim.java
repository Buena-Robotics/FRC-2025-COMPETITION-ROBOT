package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {
    private final double LIFT_SIM_P = 0.5;
    private final double LIFT_SIM_I = 0.3;
    private final double LIFT_SIM_D = 0.2;
    private final double LIFT_CARRIAGE_MASS_KG = 2.0;
    private final double LIFT_DRUM_RADIUS_METERS = Units.inchesToMeters(8);

    private final DCMotor lift_gearbox = DCMotor.getNeo550(1);
    private final ProfiledPIDController lift_controller = new ProfiledPIDController(LIFT_SIM_P, LIFT_SIM_I, LIFT_SIM_D, new TrapezoidProfile.Constraints(60, 60));

    private final ElevatorSim lift_sim = new ElevatorSim(lift_gearbox, Elevator.LIFT_MOTOR_REDUCTION, LIFT_CARRIAGE_MASS_KG, LIFT_DRUM_RADIUS_METERS, 0, Units.inchesToMeters(Elevator.ELEVATOR_MAX_HEIGHT_INCHES), false, 0, 0.00, 0.00);

    private double lift_applied_volts = 0.0;
    private boolean lift_brake_mode = true;

    public ElevatorIOSim() {
        lift_controller.setIntegratorRange(0.01, 1.0);
        lift_controller.setTolerance(0.1);
    }

    @Override public void updateInputs(final ElevatorIOInputs inputs) {
        lift_applied_volts = MathUtil.clamp(lift_controller.calculate(Units.metersToInches(lift_sim.getPositionMeters())), -12.0, 12.0);
        lift_sim.setInputVoltage(lift_applied_volts);
        lift_sim.update(0.02);

        // inputs.lift_setpoint_position_inches =
        // lift_controller.getSetpoint().position;
        inputs.lift_setpoint_position_inches = lift_controller.getGoal().position;
        inputs.lift_position_inches = Units.metersToInches(lift_sim.getPositionMeters());
        inputs.lift_velocity_inches_per_second = Units.metersToInches(lift_sim.getVelocityMetersPerSecond());
        inputs.lift_applied_volts = lift_applied_volts;
        inputs.lift_current_amps = lift_sim.getCurrentDrawAmps();
        inputs.lift_connected = true;
    }

    @Override public void setLiftPosition(double lift_setpoint_position_inches) {
        if (!lift_brake_mode)
            lift_controller.setGoal(0.0);
        else
            lift_controller.setGoal(lift_setpoint_position_inches);
    }

    @Override public void setLiftBrakeMode(final boolean brake_mode) {
        lift_brake_mode = brake_mode;
    }

    @Override public void zeroLiftPosition() {
        lift_sim.setState(0.0, 0.0);
    }
}
