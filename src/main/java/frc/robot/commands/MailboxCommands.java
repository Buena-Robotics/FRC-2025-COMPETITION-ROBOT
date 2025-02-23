package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.mailbox.Mailbox;

public class MailboxCommands {
    private MailboxCommands() {}
    public static Command triggerMailboxSpeed(final Mailbox mailbox, final DoubleSupplier speed_supplier){
        return Commands.run(() -> {
            mailbox.runSpeedSetpoint(speed_supplier.getAsDouble());
        }, mailbox);
    }
    public static Command latchOntoCoral(final Mailbox mailbox){
        return Commands.deadline(new WaitCommand(0.3), Commands.run(
            () -> { mailbox.runSpeedSetpoint(-0.1); }, mailbox));
    }
}
