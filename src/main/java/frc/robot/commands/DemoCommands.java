package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DemoCommands {
    private DemoCommands() {}

    public static Command trackApriltag() {
        return new WaitCommand(0);
    }
}
