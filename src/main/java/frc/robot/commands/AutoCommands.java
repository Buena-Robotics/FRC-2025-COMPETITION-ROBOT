package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants.ReefBranchHeight;
import frc.robot.FieldConstants.ReefBranchSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.mailbox.Mailbox;

public class AutoCommands {
    public static Command leaveCommunity(final Drive drive){
        return Commands.deadline(new WaitCommand(2), DriveCommands.driveDirection(drive, new Rotation2d(Units.degreesToRadians(0))));
    }
    public static Command singleCoral(final Drive drive, final Elevator elevator){
        return DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Right, () -> ReefBranchHeight.L2).withTimeout(6.0);
    }
    public static Command singleCoralLeftBlue(final Drive drive, final Elevator elevator, final Mailbox mailbox){
        return Commands.sequence(
            DriveCommands.gotoPose(drive, new Pose2d(5.639, 6.405, Rotation2d.fromDegrees(-120))).withTimeout(6.0),
            DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Right, () -> ReefBranchHeight.L2).withTimeout(6.0),
            MailboxCommands.triggerMailboxSpeed(mailbox, () -> -1).withTimeout(2.0)
        );
    }
    public static Command singleCoralRightBlue(final Drive drive, final Elevator elevator, final Mailbox mailbox){
        return Commands.sequence(
            DriveCommands.gotoPose(drive, new Pose2d(5.639, 2.183, Rotation2d.fromDegrees(120))).withTimeout(6.0),
            DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Right, () -> ReefBranchHeight.L2).withTimeout(6.0),
            MailboxCommands.triggerMailboxSpeed(mailbox, () -> -1).withTimeout(2.0)
        );
    }

    public static Command singleCoralLeftRed(final Drive drive, final Elevator elevator, final Mailbox mailbox){
        return Commands.sequence(
            DriveCommands.gotoPose(drive, new Pose2d(11.838, 6.405, Rotation2d.fromDegrees(60))).withTimeout(6.0),
            DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Right, () -> ReefBranchHeight.L2).withTimeout(6.0),
            MailboxCommands.triggerMailboxSpeed(mailbox, () -> -1).withTimeout(2.0)
        );
    }
    public static Command singleCoralRightRed(final Drive drive, final Elevator elevator, final Mailbox mailbox){
        return Commands.sequence(
            DriveCommands.gotoPose(drive, new Pose2d(11.838, 2.183, Rotation2d.fromDegrees(-60))).withTimeout(6.0),
            DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Right, () -> ReefBranchHeight.L2).withTimeout(6.0),
            MailboxCommands.triggerMailboxSpeed(mailbox, () -> -1).withTimeout(2.0)
        );
    }

    // 1) Grab Algae -> Coral L3 3R -> Process Algae -> Coral Station Bottom -> Coral L3 3L|Coral L2 5L
    // Coral L3 3R -> Grab Algae -> Process Algae -> Coral Station Bottom
}
