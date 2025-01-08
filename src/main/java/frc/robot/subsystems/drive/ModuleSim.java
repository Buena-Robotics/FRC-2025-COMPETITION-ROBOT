package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

public class ModuleSim implements ModuleIO {
    public static final double DRIVE_SIM_P = 0.05;
    public static final double DRIVE_SIM_D = 0.0;
    public static final double DRIVE_SIM_S = 0.0;
    public static final double DRIVE_SIM_V = 0.0789;

    public static final double TURN_SIM_P = 8.0;
    public static final double TURN_SIM_D = 0.0;

    private final SwerveModuleSimulation module_simulation;
    private final SimulatedMotorController.GenericMotorController drive_motor;
    private final SimulatedMotorController.GenericMotorController turn_motor;

    private boolean drive_closed_loop = false;
    private boolean turn_closed_loop = false;
    private final PIDController drive_controller = new PIDController(DRIVE_SIM_P, 0, DRIVE_SIM_D);
    private final PIDController turn_controller = new PIDController(TURN_SIM_P, 0, TURN_SIM_D);
    private double drive_ff_volts = 0.0;
    private double drive_applied_volts = 0.0;
    private double turn_applied_volts = 0.0;

    public ModuleSim(SwerveModuleSimulation module_simulation) {
        this.module_simulation = module_simulation;
        this.drive_motor = module_simulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(Drive.DRIVE_MOTOR_CURRENT_LIMIT));
        this.turn_motor = module_simulation
                .useGenericControllerForSteer()
                .withCurrentLimit(Amps.of(Module.TURN_MOTOR_CURRENT_LIMIT));

        // Enable wrapping for turn PID
        turn_controller.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Run closed-loop control
        if (drive_closed_loop) {
            drive_applied_volts = drive_ff_volts
                    + drive_controller.calculate(
                            module_simulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
        } else {
            drive_controller.reset();
        }
        if (turn_closed_loop) {
            turn_applied_volts = turn_controller.calculate(
                    module_simulation.getSteerAbsoluteFacing().getRadians());
        } else {
            turn_controller.reset();
        }

        // Update simulation state
        drive_motor.requestVoltage(Volts.of(drive_applied_volts));
        turn_motor.requestVoltage(Volts.of(turn_applied_volts));

        // Update drive inputs
        inputs.drive_connected = true;
        inputs.drive_position_radians =
                module_simulation.getDriveWheelFinalPosition().in(Radians);
        inputs.drive_velocity_radians_per_second =
                module_simulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
        inputs.drive_applied_volts = drive_applied_volts;
        inputs.drive_current_amps =
                Math.abs(module_simulation.getDriveMotorStatorCurrent().in(Amps));

        // Update turn inputs
        inputs.turn_connected = true;
        inputs.turn_position = module_simulation.getSteerAbsoluteFacing();
        inputs.turn_velocity_radians_per_second =
                module_simulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
        inputs.turn_applied_volts = turn_applied_volts;
        inputs.turn_current_amps =
                Math.abs(module_simulation.getSteerMotorStatorCurrent().in(Amps));

        // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
        // matter)
        inputs.odometry_timestamps = new double[] {Timer.getFPGATimestamp()};
        inputs.odometry_drive_position_radians = new double[] {inputs.drive_position_radians};
        inputs.odometry_turn_positions = new Rotation2d[] {inputs.turn_position};
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
