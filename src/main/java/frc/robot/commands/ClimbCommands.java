package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.climb.Climb;

public class ClimbCommands {
    private ClimbCommands() {}
        public static final Command triggerClimbSpeed(final Climb climb){
        return new Command(){};
    }
}
