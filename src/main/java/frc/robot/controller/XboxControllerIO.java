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
        return controller.start();
    }

    @Override public Trigger stopXBtn() {
        return controller.x();
    }

    @Override public Trigger elevatorSetpointModeBtn() {
        throw new UnsupportedOperationException("Unimplemented method 'elevatorSetpointModeBtn'");
    }

    @Override public Trigger fieldOrientedBtn() {
        throw new UnsupportedOperationException("Unimplemented method 'fieldOrientedBtn'");
    }

    @Override public Trigger driveAssistBtn() {
        throw new UnsupportedOperationException("Unimplemented method 'driveAssistBtn'");
    }

    @Override public Trigger mailboxFeedBtn() {
        throw new UnsupportedOperationException("Unimplemented method 'mailboxFeedBtn'");
    }

    @Override public Trigger flipRobotBtn() {
        throw new UnsupportedOperationException("Unimplemented method 'flipRobotBtn'");
    }

    @Override public Trigger flyToCoralStation1() {
        throw new UnsupportedOperationException("Unimplemented method 'flyToCoralStation1'");
    }

    @Override public Trigger flyToCoralStation2() {
        throw new UnsupportedOperationException("Unimplemented method 'flyToCoralStation2'");
    }

    @Override public Trigger FlyToClosestReefSide1() {
        throw new UnsupportedOperationException("Unimplemented method 'FlyToClosestReefSide1'");
    }

    @Override public Trigger FlyToClosestReefSide2() {
        throw new UnsupportedOperationException("Unimplemented method 'FlyToClosestReefSide2'");
    }
}
