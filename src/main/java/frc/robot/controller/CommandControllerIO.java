package frc.robot.controller;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public interface CommandControllerIO {
    public double getDriveXAxis();

    public double getDriveYAxis();

    public double getTurnAxis();

    public double getElevatorAxis();

    public double getMailboxAxis();

    public double getClimbAxis();

    public double getHingeAxis();

    public Trigger fullClimb();

    public Trigger algaeRelease();

    public Trigger algaeHigh();

    public Trigger algaeLow();

    public Trigger fieldOrientedBtn();

    public Trigger driveAssistBtn();

    public Trigger flipRobotBtn();

    public Trigger elevatorSetpointModeBtn();

    public Trigger mailboxFeedBtn();

    public Trigger fireDrive();

    public Trigger flyToCoralStationLeft();
    public Trigger flyToCoralStationRight();

    public Trigger flyToClosestReefLeftL2();
    public Trigger flyToClosestReefLeftL3();
    public Trigger flyToClosestReefRightL2();
    public Trigger flyToClosestReefRightL3();

    public Trigger modeTeleop();
    public Trigger modeSemiAuto();
    public Trigger modeAuto();
    public Trigger disableBologna();
}
