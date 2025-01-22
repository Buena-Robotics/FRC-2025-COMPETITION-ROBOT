package frc.robot.controller;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class SaitekControllerIO implements CommandControllerIO {
    private final Joystick joystick;

    public SaitekControllerIO(final int port) {
        this.joystick = new Joystick(0);
    }

    @Override public double getDriveXAxis() {
        return 0.0;
    }

    @Override public double getDriveYAxis() {
        return 0.0;
    }

    @Override public double getTurnAxis() {
        return 0.0;
    }

    @Override public double getElevatorAxis() {
        return joystick.getRawAxis(6) + 0.5;
    }

    @Override public double getTestAxis1() {
        return 0.0;
    }

    @Override public double getTestAxis2() {
        return 0.0;
    }

    @Override public Trigger lockGyroBtn() {
        return new Trigger(() -> joystick.getRawButton(3));
    }

    @Override public Trigger resetGyroBtn() {
        return new Trigger(() -> joystick.getRawButton(1));
    }

    @Override public Trigger stopXBtn() {
        return new Trigger(() -> joystick.getRawButton(2));
    }
}
