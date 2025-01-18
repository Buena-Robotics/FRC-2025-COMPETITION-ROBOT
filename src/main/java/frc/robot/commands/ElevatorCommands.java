package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorCommands {
    private ElevatorCommands() {}

    public static Command triggerElevatorHeight(final Elevator elevator, final DoubleSupplier height_supplier) {
        return Commands.run(
            () -> {
                elevator.runSetpoint(height_supplier.getAsDouble() * Elevator.ELEVATOR_MAX_HEIGHT_INCHES);
            }, elevator);
    }
}
