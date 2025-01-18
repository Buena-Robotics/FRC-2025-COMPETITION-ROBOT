package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    public static final double ELEVATOR_MAX_HEIGHT_INCHES = 24.0;
    public static final double LIFT_MOTOR_REDUCTION = 30.0;

    private final ElevatorIO io;
    private final Alert lift_disconnect_alert = new Alert("Disconnected lift motor", AlertType.kError);;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(final ElevatorIO io) {
        this.io = io;
    }

    @Override public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        // Update alerts
        lift_disconnect_alert.set(!inputs.lift_connected);
    }

    public void runSetpoint(final double lift_position_inches) {
        io.setLiftPosition(lift_position_inches);
    }
}
