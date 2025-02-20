package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;

public class ClimbCommands {
    private ClimbCommands() {}
    public static final Command triggerClimbSpeed(final Climb climb, final DoubleSupplier climb_speed_supplier){
        return Commands.run(() -> {
            climb.runSetpoint(climb_speed_supplier.getAsDouble());
        }, climb);
    }
}
