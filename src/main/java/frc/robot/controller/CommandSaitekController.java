package frc.robot.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class CommandSaitekController extends CommandGenericHID {
    private final Joystick saitek;

    public CommandSaitekController(final int port) {
        super(port);
        this.saitek = new Joystick(port);
    }

    public double joystickForwardAxis() {
        return saitek.getRawAxis(0);
    }

    public double joystickSideAxis() {
        return saitek.getRawAxis(1);
    }

    public double joystickRotationAxis() {
        return saitek.getRawAxis(2);
    }

    public double throttleMainAxis() {
        return saitek.getRawAxis(5);
    }

    public double throttleSliderAxis() {
        return saitek.getRawAxis(6);
    }

    public Trigger joystickDoubleTriggerFirstStage() {
        return new Trigger(() -> saitek.getRawButton(5));
    }
    public Trigger joystickDoubleTriggerSecondStage() {
        return new Trigger(() -> saitek.getRawButton(6));
    }
}
