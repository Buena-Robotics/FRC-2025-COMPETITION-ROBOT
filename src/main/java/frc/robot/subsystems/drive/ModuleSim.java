package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class ModuleSim extends ModuleIO {
    public static final double DRIVE_SIM_P = 0.05;
    public static final double DRIVE_SIM_D = 0.0;
    public static final double DRIVE_SIM_S = 0.0;
    public static final double DRIVE_SIM_V = 0.0789;

    public static final double TURN_SIM_P = 8.0;
    public static final double TURN_SIM_D = 0.0;

    private final DCMotorSim drive_sim;
    private final DCMotorSim turn_sim;

    private boolean drive_closed_loop = false;
    private boolean turn_closed_loop = false;
    private PIDController drive_controller = new PIDController(DRIVE_SIM_P, 0, DRIVE_SIM_D);
    private PIDController turn_controller = new PIDController(TURN_SIM_P, 0, TURN_SIM_D);
    private double drive_ff_volts = 0.0;
    private double drive_applied_volts = 0.0;
    private double turn_applied_volts = 0.0;

    public ModuleSim(int module) {
        super(module);
        // Create drive and turn sim models
        drive_sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(Drive.DRIVE_GEARBOX, 0.025, Drive.DRIVE_MOTOR_REDUCTION),
                Drive.DRIVE_GEARBOX);
        turn_sim = new DCMotorSim(
                LinearSystemId.createDCMotorSystem(TURN_GEARBOX, 0.004, TURN_MOTOR_REDUCTION),
                TURN_GEARBOX);

        // Enable wrapping for turn PID
        turn_controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (drive_closed_loop) {
            drive_applied_volts = drive_ff_volts + drive_controller.calculate(drive_sim.getAngularVelocityRadPerSec());
        } else {
            drive_controller.reset();
        }
        if (turn_closed_loop) {
            turn_applied_volts = turn_controller.calculate(turn_sim.getAngularPositionRad());
        } else {
            turn_controller.reset();
        }

        // Update simulation state
        drive_sim.setInputVoltage(MathUtil.clamp(drive_applied_volts, -12.0, 12.0));
        turn_sim.setInputVoltage(MathUtil.clamp(turn_applied_volts, -12.0, 12.0));
        drive_sim.update(0.02);
        turn_sim.update(0.02);

        // Update drive inputs
        inputs.drive_connected = true;
        inputs.drive_position_radians = drive_sim.getAngularPositionRad();
        inputs.drive_velocity_radians_per_second = drive_sim.getAngularVelocityRadPerSec();
        inputs.drive_applied_volts = drive_applied_volts;
        inputs.drive_current_amps = Math.abs(drive_sim.getCurrentDrawAmps());

        // Update turn inputs
        inputs.turn_connected = true;
        inputs.turn_position = new Rotation2d(turn_sim.getAngularPositionRad());
        inputs.turn_velocity_radians_per_second = turn_sim.getAngularVelocityRadPerSec();
        inputs.turn_applied_volts = turn_applied_volts;
        inputs.turn_current_amps = Math.abs(turn_sim.getCurrentDrawAmps());

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometry_timestamps = new double[] { Timer.getFPGATimestamp() };
        inputs.odometry_drive_position_radians = new double[] { inputs.drive_position_radians };
        inputs.odometry_turn_positions = new Rotation2d[] { inputs.turn_position };
    }

    @Override
    public void setDriveOpenLoop(double output) {
        drive_closed_loop = false;
        drive_applied_volts = output;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turn_closed_loop = false;
        turn_applied_volts = output;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        drive_closed_loop = true;
        drive_ff_volts = DRIVE_SIM_S * Math.signum(velocityRadPerSec) + DRIVE_SIM_V * velocityRadPerSec;
        drive_controller.setSetpoint(velocityRadPerSec);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turn_closed_loop = true;
        turn_controller.setSetpoint(rotation.getRadians());
    }
}