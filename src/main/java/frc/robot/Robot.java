// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.networktables.ConnectionInfo;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Config.RobotMode;
import frc.robot.Config.RobotType;
import frc.robot.util.BatteryTracker;
import frc.robot.util.CanTracker;
import frc.robot.util.MemTracker;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;

import org.ironmaple.simulation.SimulatedArena;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private static final Map<Integer, String> GIT_DIRTY_MSG_MAP = Map.of(0, "All changes committed", 1, "Uncomitted changes");

    private final RobotContainer robot_container;
    private Command autonomous_command;

    private final Timer can_error_timer = new Timer();
    private final Timer can_error_timer_initial = new Timer();
    private final Timer disabled_timer = new Timer();

    private final Alert can_error_alert = new Alert("CAN errors detected, robot may not be controllable.", AlertType.kError);
    private final Alert log_reciever_queue_alert = new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.kError);
    private final Alert log_no_file_alert = new Alert("No log path set for current robot. Data will NOT be logged.", AlertType.kWarning);
    private final Alert same_battery_alert = new Alert("The battery has not been changed since the last match.", AlertType.kWarning);
    private final Alert low_battery_alert = new Alert("Battery voltage is very low, consider turning off the robot or replacing the battery.", AlertType.kWarning);
    private final Alert low_memory_alert = new Alert("Running out of memory, java program may crash soon", AlertType.kWarning);

    public Robot() {
        DriverStation.silenceJoystickConnectionWarning(true);
        // Record metadata
        Logger.recordMetadata("RobotType", Config.ROBOT_TYPE.toString());
        Logger.recordMetadata("RobotMode", Config.ROBOT_MODE.toString());
        System.out.println("[Init] Scanning battery");
        Logger.recordMetadata("BatteryName", "BAT-" + BatteryTracker.scanBattery(1.5));
        System.out.println("[Init] Starting AdvantageKit");
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("GitDirty", GIT_DIRTY_MSG_MAP.getOrDefault(BuildConstants.DIRTY, "Unknown"));

        // Set up data receivers & replay source
        switch (Config.ROBOT_MODE) {
            case REAL:
                initializeRealAkit();
                break;
            case SIM:
                initializeSimAkit();
                break;
            case REPLAY:
                initializeReplayAkit();
                break;
        }

        // Initialize URCL
        // Logger.registerURCL(URCL.startExternal());

        // Start AdvantageKit logger
        Logger.start();

        same_battery_alert.set(BatteryTracker.sameBatteryCheck());

        // Log active commands
        Map<String, Integer> command_counts = new HashMap<>();
        BiConsumer<Command, Boolean> log_commmand_function = (Command command, Boolean active) -> {
            String name = command.getName();
            int count = command_counts.getOrDefault(name, 0) + (active ? 1 : -1);
            command_counts.put(name, count);
            Logger.recordOutput("CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
        CommandScheduler.getInstance().onCommandInitialize(acceptLogCommand(log_commmand_function, true));
        CommandScheduler.getInstance().onCommandFinish(acceptLogCommand(log_commmand_function, false));
        CommandScheduler.getInstance().onCommandInterrupt(acceptLogCommand(log_commmand_function, false));

        if (Config.ROBOT_MODE == RobotMode.SIM)
            initializeDriverstationSim();

        // Start timers
        can_error_timer.reset();
        can_error_timer.start();
        can_error_timer_initial.reset();
        can_error_timer_initial.start();
        disabled_timer.reset();
        disabled_timer.start();

        robot_container = new RobotContainer();
        // PathfindingCommand.warmupCommand().schedule();
    }

    @Override public void robotPeriodic() {
        // Switch thread to high priority to improve loop timing
        Threads.setCurrentThreadPriority(true, 99);

        CommandScheduler.getInstance().run();

        // Check logging fault
        log_reciever_queue_alert.set(Logger.getReceiverQueueFault());

        // Update CAN error alert
        can_error_alert.set(CanTracker.hasCanErrors(can_error_timer, can_error_timer_initial));

        // Update low battery alert
        if (DriverStation.isEnabled())
            disabled_timer.reset();
        low_battery_alert.set(BatteryTracker.isLowVoltage(disabled_timer));

        low_memory_alert.set(MemTracker.highMemoryUsage());
        Logger.recordOutput("MemoryUsage", MemTracker.memoryUsage());

        // Log list of NT clients
        List<String> clientNames = new ArrayList<>();
        List<String> clientAddresses = new ArrayList<>();
        for (ConnectionInfo client : NetworkTableInstance.getDefault().getConnections()) {
            clientNames.add(client.remote_id);
            clientAddresses.add(client.remote_ip);
        }
        Logger.recordOutput("NTClients/Names", clientNames.toArray(new String[clientNames.size()]));
        Logger.recordOutput("NTClients/Addresses", clientAddresses.toArray(new String[clientAddresses.size()]));

        BatteryTracker.tryWriteBatteryName();

        // Return to normal thread priority
        Threads.setCurrentThreadPriority(false, 10);
    }

    @Override public void disabledInit() {
        robot_container.resetSimulationField();
    }

    @Override public void disabledPeriodic() {}

    @Override public void disabledExit() {}

    @Override public void autonomousInit() {
        autonomous_command = robot_container.getAutonomousCommand();

        if (autonomous_command != null) {
            autonomous_command.schedule();
        }
    }

    @Override public void autonomousPeriodic() {}

    @Override public void autonomousExit() {}

    @Override public void teleopInit() {
        if (autonomous_command != null) {
            autonomous_command.cancel();
        }
    }

    @Override public void teleopPeriodic() {}

    @Override public void teleopExit() {}

    @Override public void simulationPeriodic() {
        SimulatedArena.getInstance().simulationPeriodic();
        robot_container.displaySimFieldToAdvantageScope();
    }

    @Override public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override public void testPeriodic() {}

    @Override public void testExit() {}

    private void initializeRealAkit() {
        // Running on a real robot, log to a USB stick ("/U/logs")
        final String folder = Config.log_folders_map.get(Config.ROBOT_TYPE);
        if (folder != null)
            Logger.addDataReceiver(new WPILOGWriter(folder));
        else
            log_no_file_alert.set(true);
        Logger.addDataReceiver(new NT4Publisher());
        if (Config.ROBOT_TYPE == RobotType.ROBOT_2025_COMPETION)
            LoggedPowerDistribution.getInstance(9, ModuleType.kRev);
    }

    private void initializeSimAkit() {
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
    }

    private void initializeReplayAkit() {
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        final String log_path = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(log_path));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(log_path, "_sim")));
    }

    private void initializeDriverstationSim() {
        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
        DriverStationSim.setFmsAttached(Config.ROBOT_TYPE == RobotType.ROBOT_2025_COMPETION);
        DriverStationSim.setEventName("SimEvent");
        DriverStationSim.setEnabled(true);
    }

    private static Consumer<Command> acceptLogCommand(BiConsumer<Command, Boolean> log_commmand_function, boolean u) {
        return (Command command) -> {
            log_commmand_function.accept(command, u);
        };
    }
}
