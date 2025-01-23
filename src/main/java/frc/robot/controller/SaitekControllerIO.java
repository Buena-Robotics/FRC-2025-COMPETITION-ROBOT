package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        return controller.throttleSliderAxis() + 0.5;
    }

    @Override public double getMailboxAxis() {
        return 0.0;
    }

    @Override public Trigger lockGyroBtn() {
        return controller.button(3);
    }

    @Override public Trigger resetGyroBtn() {
        return controller.button(1);
    }

    @Override public Trigger stopXBtn() {
        return controller.button(2);
    }
}
