package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants.ReefBranchHeight;
import frc.robot.FieldConstants.ReefBranchSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;

public class AutoCommands {
    public static Command leaveCommunity(final Drive drive){
        return Commands.deadline(new WaitCommand(10), DriveCommands.driveDirection(drive, new Rotation2d(Units.degreesToRadians(0))));
    }
    public static Command singleCoral(final Drive drive, final Elevator elevator){
        return DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Right, () -> ReefBranchHeight.L2).withTimeout(6.0);
    }

    // 1) Grab Algae -> Coral L3 3R -> Process Algae -> Coral Station Bottom -> Coral L3 3L|Coral L2 5L
    // Coral L3 3R -> Grab Algae -> Process Algae -> Coral Station Bottom
}
