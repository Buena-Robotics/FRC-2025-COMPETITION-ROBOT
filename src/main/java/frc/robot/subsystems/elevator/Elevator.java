package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private final Alert lift_disconnect_alert;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(final ElevatorIO io){
        this.io = io;
        this.lift_disconnect_alert = new Alert("Disconnected lift motor", AlertType.kError);
    }
}
