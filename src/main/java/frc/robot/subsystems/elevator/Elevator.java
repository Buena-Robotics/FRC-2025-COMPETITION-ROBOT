package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {
    public static final double ELEVATOR_MAX_HEIGHT_INCHES = 17.0;
    public static final double ELEVATOR_BASE_HEIGHT = 36.0;
    public static final double LIFT_MOTOR_REDUCTION = 20.0;
    public static final double ELEVATOR_COAST_FALL_TIME_SECONDS = 4.0;

    private final Mechanism2d elevator_mech;
    private final MechanismLigament2d elevator_mech_base;
    private final MechanismLigament2d elevator_mech_shaft;
    private final MechanismLigament2d elevator_mech_mailbox;

    private final ElevatorIO io;
    private final Alert lift_disconnect_alert = new Alert("Disconnected lift motor", AlertType.kError);;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final SysIdRoutine sys_id;

    @AutoLogOutput(key = "Elevator/BrakeModeEnabled")
    private boolean brake_mode_enabled = true;

    public Elevator(final ElevatorIO io) {
        this.io = io;
        this.elevator_mech = new Mechanism2d(Units.inchesToMeters(29), Units.inchesToMeters(ELEVATOR_MAX_HEIGHT_INCHES + ELEVATOR_BASE_HEIGHT), new Color8Bit("#202020"));
        this.elevator_mech_base = new MechanismLigament2d("ElevatorBase", Units.inchesToMeters(ELEVATOR_BASE_HEIGHT), 90, 4, new Color8Bit("#FF50FF"));
        this.elevator_mech_shaft = new MechanismLigament2d("ElevatorShaft", 0, 0, 4, new Color8Bit("#801010"));
        this.elevator_mech_mailbox = new MechanismLigament2d("Mailbox", Units.inchesToMeters(12), -90, 2, new Color8Bit("#FF5050"));
        this.elevator_mech.getRoot("ElevatorMech", Units.inchesToMeters(5), 0).append(elevator_mech_base).append(elevator_mech_shaft).append(elevator_mech_mailbox);
        SmartDashboard.putData("ElevatorMechData", elevator_mech);

        this.sys_id = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.01).per(Second), Volts.of(0.01), Seconds.of(60), (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        brake_mode_enabled = DriverStation.isEnabled();
        io.setLiftBrakeMode(brake_mode_enabled);
        elevator_mech_shaft.setLength(Units.inchesToMeters(inputs.lift_position_inches));

        // Update alerts
        lift_disconnect_alert.set(!inputs.lift_connected);
        Logger.recordOutput("Elevator/Error", Math.abs(inputs.lift_position_inches - inputs.lift_setpoint_position_inches));
    }

    public void runCharacterization(final double output) {
        io.setLiftOpenLoop(output);
    }

    public Command sysIdQuasistatic(final SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sys_id.quasistatic(direction));
    }

    public Command sysIdDynamic(final SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sys_id.dynamic(direction));
    }

    public double getFFCharacterizationVelocity() {
        return inputs.lift_velocity_inches_per_second;
    }

    public void runSetpoint(final double lift_position_inches) {
        io.setLiftPosition(lift_position_inches);
    }

    public static enum ElevatorSetpoint {
        BOTTOM(0.0), TOP(Elevator.ELEVATOR_MAX_HEIGHT_INCHES);

        private double setpoint_inches = 0.0;

        private ElevatorSetpoint(final double setpoint_inches) {
            this.setpoint_inches = setpoint_inches;
        }

        public double getValue() {
            return setpoint_inches;
        }
    }
}
