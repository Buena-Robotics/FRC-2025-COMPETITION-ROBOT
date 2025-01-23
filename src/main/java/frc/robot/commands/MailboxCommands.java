package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.mailbox.Mailbox;

public class MailboxCommands {
    private MailboxCommands() {}
    public static final Command triggerMailboxSpeed(final Mailbox mailbox, final DoubleSupplier speed_supplier){
        return Commands.run(() -> {
            mailbox.runSpeedSetpoint(speed_supplier.getAsDouble());
        }, mailbox);
    }
}
