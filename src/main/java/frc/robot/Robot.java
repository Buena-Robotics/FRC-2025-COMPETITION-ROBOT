// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
	private Command autonomous_command;
	private final RobotContainer robot_container;

	public Robot() {
		// Set up data receivers & replay source
		switch (Config.ROBOT_MODE) {
			case REAL:
				// Running on a real robot, log to a USB stick ("/U/logs")
				Logger.addDataReceiver(new WPILOGWriter());
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case SIM:
				// Running a physics simulator, log to NT
				Logger.addDataReceiver(new NT4Publisher());
				break;
			case REPLAY:
				// Replaying a log, set up replay source
				setUseTiming(false); // Run as fast as possible
				String logPath = LogFileUtil.findReplayLog();
				Logger.setReplaySource(new WPILOGReader(logPath));
				Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
				break;
		}
	
		// Start AdvantageKit logger
		Logger.start();

		robot_container = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		// Switch thread to high priority to improve loop timing
		Threads.setCurrentThreadPriority(true, 99);

		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled commands, running already-scheduled commands, removing
		// finished or interrupted commands, and running subsystem periodic() methods.
		// This must be called from the robot's periodic block in order for anything in
		// the Command-based framework to work.
		CommandScheduler.getInstance().run();

		// Return to normal thread priority
		Threads.setCurrentThreadPriority(false, 10);
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		autonomous_command = robot_container.getAutonomousCommand();

		if (autonomous_command != null) {
			autonomous_command.schedule();
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
	}

	@Override
	public void teleopInit() {
		if (autonomous_command != null) {
			autonomous_command.cancel();
		}
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}
