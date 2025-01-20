package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {
    public static final double WINCH_MOTOR_REDUCTION = 1.0;

    private final ClimbIO io;
    private final Alert winch_disconnect_alert = new Alert("Disconnected winch motor", AlertType.kError);;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    public Climb(final ClimbIO io) {
        this.io = io;
    }

    @Override public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        // Update alerts
        winch_disconnect_alert.set(!inputs.winch_connected);
    }

    public void runSetpoint(final double winch_speed) {
        io.setWinchSpeed(winch_speed);
    }
}
