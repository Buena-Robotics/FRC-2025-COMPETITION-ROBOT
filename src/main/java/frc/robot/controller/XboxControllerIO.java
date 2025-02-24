package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class XboxControllerIO implements CommandControllerIO {
    private final CommandXboxController controller;

    public XboxControllerIO(final int port) {
        controller = new CommandXboxController(port);
    }

    @Override public double getDriveXAxis() {
        return controller.getLeftX();
    }

    @Override public double getDriveYAxis() {
        return controller.getLeftY();
    }

    @Override public double getTurnAxis() {
        return controller.getRightX();
    }

    @Override public double getElevatorAxis() {
        return controller.getRightTriggerAxis();
    }

    @Override public double getMailboxAxis() {
        return controller.rightBumper().getAsBoolean() ? -0.5 : 0.0;
    }

    @Override public double getClimbAxis() {
        return controller.povDown().getAsBoolean() ? -0.25 : controller.povUp().getAsBoolean() ? 0.25 : 0.0;
    }

    @Override public Trigger resetGyroBtn() {
        return new Trigger(() -> false);
    }

    @Override public Trigger stopXBtn() {
        return new Trigger(() -> false);
    }

    @Override public Trigger elevatorSetpointModeBtn() {
        return controller.povLeft();
    }

    @Override public Trigger fieldOrientedBtn() {
        return controller.a();
    }

    @Override public Trigger driveAssistBtn() {
        return controller.b();
    }

    @Override public Trigger mailboxFeedBtn() {
        return new Trigger(() -> false);
    }

    @Override public Trigger flipRobotBtn() {
        return new Trigger(() -> false);
    }

    @Override public Trigger flyToCoralStation1() {
        return new Trigger(() -> false);
    }

    @Override public Trigger flyToCoralStation2() {
        return new Trigger(() -> false);
    }

    @Override public Trigger FlyToClosestReefSide1() {
        return controller.back();
    }

    @Override public Trigger FlyToClosestReefSide2() {
        return controller.start();
    }
}
