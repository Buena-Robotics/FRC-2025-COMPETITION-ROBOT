package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.controller.CommandSaitekController.Button;

public class SaitekControllerIO implements CommandControllerIO {
    private final CommandSaitekController controller;

    public SaitekControllerIO(final int port) {
        this.controller = new CommandSaitekController(0);
    }

    @Override public double getDriveXAxis() {
        return controller.joystickSideAxis();
    }

    @Override public double getDriveYAxis() {
        return controller.joystickForwardAxis();
    }

    @Override public double getTurnAxis() {
        return controller.joystickRotationAxis();
    }

    @Override public double getElevatorAxis() {
        return 1.0 - ((controller.throttleMainAxis() + 1.0) / 2.0);
    }

    @Override public double getMailboxAxis() {
        return controller.getTrigger(Button.I).getAsBoolean() ? -1.0 : 0.0;
    }

    @Override public double getClimbAxis() {
        return controller.povDown().getAsBoolean() ? -0.25 : controller.povUp().getAsBoolean() ? 0.25 : 0.0;
    }

    @Override public double getHingeAxis() {
        return (controller.throttleSliderAxis() + 1.0) / 2.0;

    }

    @Override public Trigger resetGyroBtn() {
        return controller.getTrigger(Button.DOUBLE_TRIGGER_1);
    }

    @Override public Trigger stopXBtn() {
        return controller.getTrigger(Button.DOUBLE_TRIGGER_2);
    }

    @Override public Trigger elevatorSetpointModeBtn() {
        return controller.getTrigger(Button.D);
    }

    @Override public Trigger fieldOrientedBtn() {
        return controller.getTrigger(Button.FIRE);
    }

    @Override public Trigger driveAssistBtn() {
        return controller.getTrigger(Button.B);
    }

    @Override public Trigger mailboxFeedBtn() {
        return controller.getTrigger(Button.I);
    }

    @Override public Trigger flipRobotBtn() {
        return controller.getTrigger(Button.C);
    }

    @Override public Trigger flyToCoralStation1() {
        return controller.getTrigger(Button.T1);
    }

    @Override public Trigger flyToCoralStation2() {
        return controller.getTrigger(Button.T2);
    }

    @Override public Trigger FlyToClosestReefSide1() {
        return controller.getTrigger(Button.T3);
    }

    @Override public Trigger FlyToClosestReefSide2() {
        return controller.getTrigger(Button.T4);
    }

    @Override public Trigger algaeRelease() {
        return new Trigger(() -> false);
    }

    @Override public Trigger algaeHigh() {
        return controller.getTrigger(Button.RESET);
    }

    @Override public Trigger algaeLow() {
        return controller.getTrigger(Button.START_STOP);
    }
}
