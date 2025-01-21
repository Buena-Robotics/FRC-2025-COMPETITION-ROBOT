package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.mailbox.Mailbox;

public class MailboxCommands {
    private MailboxCommands() {}
    public static final Command triggerMailboxSpeed(final Mailbox mailbox){
        return new Command(){};
    }
}
