package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface CommandControllerIO {
    public double getDriveXAxis();

    public double getDriveYAxis();

    public double getTurnAxis();

    public double getElevatorAxis();

    public double getTestAxis1();

    public double getTestAxis2();

    public Trigger lockGyroBtn();

    public Trigger resetGyroBtn();

    public Trigger stopXBtn();
}
