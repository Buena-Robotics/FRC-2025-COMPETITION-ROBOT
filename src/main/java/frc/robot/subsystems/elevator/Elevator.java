package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public static final double ELEVATOR_MAX_HEIGHT_INCHES = 17.0;
    public static final double LIFT_MOTOR_REDUCTION = 20.0;
    public static final double ELEVATOR_COAST_FALL_TIME_SECONDS = 4.0;

    private final Mechanism2d elevator_mech;
    private final MechanismLigament2d elevator_mech_shaft;
    private final MechanismLigament2d elevator_mech_mailbox;

    private final ElevatorIO io;
    private final Alert lift_disconnect_alert = new Alert("Disconnected lift motor", AlertType.kError);;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(final ElevatorIO io) {
        this.io = io;
        this.elevator_mech = new Mechanism2d(29 + 4, 17 + 2, new Color8Bit("#202020"));
        this.elevator_mech_shaft = new MechanismLigament2d("ElevatorShaft", 17, 90, 4, new Color8Bit("#801010"));
        this.elevator_mech_mailbox = new MechanismLigament2d("Mailbox", 12, -90, 3, new Color8Bit("#FF5050"));
        this.elevator_mech.getRoot("ElevatorMech", 5, 0).append(elevator_mech_shaft).append(elevator_mech_mailbox);
        SmartDashboard.putData("ElevatorMechData", elevator_mech);
    }

    @Override public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        io.setLiftBrakeMode(DriverStation.isEnabled());
        elevator_mech_shaft.setLength(inputs.lift_position_inches + 1);

        // Update alerts
        lift_disconnect_alert.set(!inputs.lift_connected);
    }

    public void runSetpoint(final double lift_position_inches) {
        io.setLiftPosition(lift_position_inches);
    }
}
