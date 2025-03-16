package frc.robot.subsystems.mailbox;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.elevator.Elevator;

public class MailboxIOSim implements MailboxIO {
    private static final double SIM_SHOOTER_P = 0.03;
    private static final double SIM_SHOOTER_D = 0.0;

    private final DCMotor shooter_gearbox = DCMotor.getNEO(1);
    private final DCMotorSim shooter_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(shooter_gearbox, 0.004, 1), shooter_gearbox);
    private final PIDController shooter_controller = new PIDController(SIM_SHOOTER_P, 0, SIM_SHOOTER_D);

    private double shooter_applied_volts = 0.0;
    private boolean open_loop = false;

    private final SwerveDriveSimulation drive_simulation;
    private final Elevator elevator;
    private final IntakeSimulation intake_simulation;

    public MailboxIOSim(final SwerveDriveSimulation drive_simulation, final Elevator elevator) {
        this.drive_simulation = drive_simulation;
        this.elevator = elevator;
        this.intake_simulation = IntakeSimulation.InTheFrameIntake(
            // Specify the type of game pieces that the intake can collect
            "Coral",
            // Specify the drivetrain to which this intake is attached
            drive_simulation,
            // Width of the intake
            Meters.of(0.4),
            // The intake is mounted on the back side of the chassis
            IntakeSimulation.IntakeSide.BACK,
            // The intake can hold up to 1 coral
            1);
    }

    @Override public void updateInputs(final MailboxIOInputs inputs) {
        if (!open_loop) {
            shooter_applied_volts = MathUtil.clamp(shooter_controller.calculate(shooter_sim.getAngularPositionRad()), -12.0, 12.0);
            shooter_sim.setInputVoltage(shooter_applied_volts);
        } else {
            if (Math.abs(shooter_applied_volts) <= 0.01) {
                shooter_sim.setInputVoltage(0);
                shooter_sim.setAngularVelocity(0);
            } else {
                shooter_sim.setInputVoltage(shooter_applied_volts);
            }
        }

        shooter_sim.update(0.02);

        inputs.shooter_position_radians = shooter_sim.getAngularPositionRad();
        inputs.shooter_velocity_radians_per_second = shooter_sim.getAngularVelocityRadPerSec();
        inputs.shooter_applied_volts = shooter_sim.getInputVoltage();
        inputs.shooter_current_amps = shooter_sim.getCurrentDrawAmps();
        inputs.shooter_connected = true;

        inputs.coral_beambreak_connected = true;
        inputs.coral_beam_broken = intake_simulation.getGamePiecesAmount() == 1 && shooter_sim.getAngularPositionRad() >= -10;

        if (inputs.shooter_position_radians <= Mailbox.CORAL_END_POSITION) {
            resetPosition();
            intake_simulation.removeObtainedGamePieces(SimulatedArena.getInstance());
            SimulatedArena.getInstance()
                .addGamePieceProjectile(new ReefscapeCoralOnFly(
                    // Obtain robot position from drive simulation
                    drive_simulation.getSimulatedDriveTrainPose().getTranslation(),
                    // The scoring mechanism is installed at (0.46, 0) (meters) on the robot
                    elevator.robotToElevator().getTranslation().toTranslation2d(),
                    // Obtain robot speed from drive simulation
                    drive_simulation.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
                    // Obtain robot facing from drive simulation
                    drive_simulation.getSimulatedDriveTrainPose().getRotation(),
                    // The height at which the coral is ejected
                    elevator.virtualCameraPosition().getTranslation().getMeasureZ(),
                    // The initial speed of the coral
                    MetersPerSecond.of(2),
                    // The coral is ejected at a 35-degree slope
                    Degrees.of(-10)));
        }
    }

    @Override public void resetPosition() {
        shooter_applied_volts = 0.0;
        shooter_sim.setState(0.0, 0.0);
    }

    @Override public void setShooterOpenLoop(final double output) {
        open_loop = true;
        shooter_applied_volts = output;
    }

    @Override public void setShooterSpeed(final double shooter_speed) {
        open_loop = true;
        shooter_applied_volts = shooter_speed * 12.0;
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
