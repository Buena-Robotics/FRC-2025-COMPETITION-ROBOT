package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface CommandControllerIO {
    public double getDriveXAxis();

    public double getDriveYAxis();

    public double getTurnAxis();

    public double getElevatorAxis();

    public double getMailboxAxis();

    public double getClimbAxis();

    public Trigger fieldOrientedBtn();

    public Trigger driveAssistBtn();

    public Trigger flipRobotBtn();

    public Trigger resetGyroBtn();

    public Trigger stopXBtn();

    public Trigger elevatorSetpointModeBtn();

    public Trigger mailboxFeedBtn();

    public Trigger flyToCoralStation1();

    public Trigger flyToCoralStation2();

    public Trigger FlyToClosestReefSide1();

    public Trigger FlyToClosestReefSide2();
}
