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
        return controller.povDown().getAsBoolean() ? 0.5 : controller.povUp().getAsBoolean() ? -0.25 : 0.0;
    }

    @Override public double getHingeAxis() {
        return (controller.throttleSliderAxis() + 1.0) / 2.0;

    }

    @Override public Trigger fullClimb() {
        return controller.getTrigger(Button.FIRE);
    }

    @Override public Trigger elevatorSetpointModeBtn() {
        return controller.getTrigger(Button.D);
    }

    @Override public Trigger fieldOrientedBtn() {
        return controller.getTrigger(Button.B);
    }

    @Override public Trigger driveAssistBtn() {
        return new Trigger(() -> false);
    }

    @Override public Trigger mailboxFeedBtn() {
        return controller.getTrigger(Button.I);
    }

    @Override public Trigger flipRobotBtn() {
        return controller.getTrigger(Button.C);
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

    @Override public Trigger fireDrive() {
        return controller.getTrigger(Button.DOUBLE_TRIGGER_1);
    }

    @Override public Trigger flyToCoralStationLeft() {
        return new Trigger(() -> false);
    }

    @Override public Trigger flyToCoralStationRight() {
        return new Trigger(() -> false);
    }

    @Override public Trigger flyToClosestReefLeftL2() {
        return controller.getTrigger(Button.T2);
    }

    @Override public Trigger flyToClosestReefLeftL3() {
        return controller.getTrigger(Button.T1);
    }

    @Override public Trigger flyToClosestReefRightL2() {
        return controller.getTrigger(Button.T4);
    }

    @Override public Trigger flyToClosestReefRightL3() {
        return controller.getTrigger(Button.T3);
    }

    @Override public Trigger modeTeleop() {
        return controller.getTrigger(Button.MODE_UP);
    }

    @Override public Trigger modeSemiAuto() {
        return controller.getTrigger(Button.MODE_MIDDLE);
    }

    @Override public Trigger modeAuto() {
        return controller.getTrigger(Button.MODE_DOWN);
    }
}
