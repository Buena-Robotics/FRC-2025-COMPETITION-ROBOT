package frc.robot.subsystems.mailbox;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Mailbox extends SubsystemBase {
    private final MailboxIO io;
    private final Alert shooter_disconnect_alert = new Alert("Disconnected shooter motor", AlertType.kError);;
    private final MailboxIOInputsAutoLogged inputs = new MailboxIOInputsAutoLogged();

    public Mailbox(final MailboxIO io) {
        this.io = io;
    }

    @Override public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Mailbox", inputs);

        shooter_disconnect_alert.set(!inputs.shooter_connected);
    }

    public void runSpeedSetpoint(final double shooter_speed) {
        io.setShooterSpeed(shooter_speed);
    }

    public void runVelocitySetpoint(final double shooter_velocity_radians_per_second) {
        io.setShooterVelocity(shooter_velocity_radians_per_second);
    }
}
