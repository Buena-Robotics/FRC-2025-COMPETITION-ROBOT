// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeReefSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config.RobotMode;
import frc.robot.Config.RobotType;
import frc.robot.FieldConstants.ReefBranchHeight;
import frc.robot.FieldConstants.ReefBranchSide;
import frc.robot.commands.AutoCommands;
import frc.robot.commands.ClimbCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.MailboxCommands;
import frc.robot.controller.CommandControllerIO;
import frc.robot.controller.SaitekControllerIO;
import frc.robot.controller.XboxControllerIO;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOReal;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.Elevator.ElevatorSetpoint;
import frc.robot.subsystems.mailbox.Mailbox;
import frc.robot.subsystems.mailbox.MailboxIO;
import frc.robot.subsystems.mailbox.MailboxIOReal;
import frc.robot.subsystems.mailbox.MailboxIOSim;
import frc.robot.subsystems.vision.Cameras;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionIOPhotonSim;
import frc.robot.util.ArenaSchool2025Reefscape;

public class RobotContainer {
    // Subsystems
    @SuppressWarnings("unused")
    private final Vision vision;
    private final Drive drive;
    private final SwerveDriveSimulation drive_simulation = Config.ROBOT_MODE == RobotMode.SIM ?
        new SwerveDriveSimulation(Drive.MAPLE_SIM_CONFIG, new Pose2d(3, 3, new Rotation2d())) :
        null;
    private final Elevator elevator;
    private final Climb climb;
    private final Mailbox mailbox;
    private final Trigger coral_waiting_trigger;

    // Controller
    private final CommandControllerIO controller = Config.ROBOT_MODE == RobotMode.SIM ? new XboxControllerIO(0) : new SaitekControllerIO(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> auto_chooser;

    public RobotContainer() {
        switch (Config.ROBOT_MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                this.drive = new Drive(
                    new GyroIOPigeon2() {},
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3));

                this.vision = new Vision(
                    drive::addVisionMeasurement, drive,
                    new VisionIOPhoton(Cameras.cameras[0]),
                    new VisionIOPhoton(Cameras.cameras[1]),
                    new VisionIOPhoton(Cameras.cameras[2]));

                this.elevator = new Elevator(new ElevatorIOReal() {}, drive::getPose);
                this.climb = new Climb(new ClimbIOReal() {});
                this.mailbox = new Mailbox(new MailboxIOReal() {}, elevator, drive::getPose);
                break;
            case SIM:
                // create a maple-sim swerve drive simulation instance
                // add the simulated drivetrain to the simulation field
                if (Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL)
                    SimulatedArena.overrideInstance(new ArenaSchool2025Reefscape());
                SimulatedArena.getInstance().addDriveTrainSimulation(drive_simulation);
                resetSimulationField();

                // Sim robot, instantiate physics sim IO implementations
                this.drive = new Drive(
                    new GyroSim(drive_simulation.getGyroSimulation()),
                    new ModuleIOSim(drive_simulation.getModules()[0]),
                    new ModuleIOSim(drive_simulation.getModules()[1]),
                    new ModuleIOSim(drive_simulation.getModules()[2]),
                    new ModuleIOSim(drive_simulation.getModules()[3]));
                // new VisionIOPhotonSim(Cameras.cameras[0]), new
                // VisionIOPhotonSim(Cameras.cameras[1])
                this.vision = new Vision(drive::addVisionMeasurement, drive,
                    new VisionIOPhotonSim(Cameras.cameras[0],
                        drive_simulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonSim(Cameras.cameras[1],
                        drive_simulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonSim(Cameras.cameras[2],
                        drive_simulation::getSimulatedDriveTrainPose));

                this.elevator = new Elevator(new ElevatorIOSim(), drive::getPose);
                this.climb = new Climb(new ClimbIOSim());
                this.mailbox = new Mailbox(new MailboxIOSim(drive_simulation, elevator), elevator, drive::getPose);
                break;
            default:
                // Replayed robot, disable IO implementations
                this.drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                this.vision = new Vision(drive::addVisionMeasurement, drive, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
                this.elevator = new Elevator(new ElevatorIO() {}, drive::getPose);
                this.climb = new Climb(new ClimbIO() {});
                this.mailbox = new Mailbox(new MailboxIO() {}, elevator, drive::getPose);
                break;
        }

        coral_waiting_trigger = new Trigger(mailbox::coralWaiting);

        intializePathplannerAutoCommands();

        // Set up auto routines
        auto_chooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        auto_chooser.addOption("Leave Community", AutoCommands.leaveCommunity(drive));

        if (Config.TUNING_PID_LOOPS) {
            auto_chooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
            auto_chooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
            auto_chooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            auto_chooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

            auto_chooser.addOption("Elevator Simple FF Characterization", ElevatorCommands.feedforwardCharacterization(elevator));
            auto_chooser.addOption("Elevator SysId (Quasistatic Forward)", elevator.liftSysIdQuasistatic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Elevator SysId (Quasistatic Reverse)", elevator.liftSysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            auto_chooser.addOption("Elevator SysId (Dynamic Forward)", elevator.liftSysIdDynamic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Elevator SysId (Dynamic Reverse)", elevator.liftSysIdDynamic(SysIdRoutine.Direction.kReverse));

            auto_chooser.addOption("Mailbox Simple FF Characterization", MailboxCommands.feedforwardCharacterization(mailbox));
            auto_chooser.addOption("Mailbox SysId (Quasistatic Forward)", mailbox.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Mailbox SysId (Quasistatic Reverse)", mailbox.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
            auto_chooser.addOption("Mailbox SysId (Dynamic Forward)", mailbox.sysIdDynamic(SysIdRoutine.Direction.kForward));
            auto_chooser.addOption("Mailbox SysId (Dynamic Reverse)", mailbox.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        }
        // Set up SysId routines

        configureBindings();
    }

    private boolean elevator_setpoint_mode = false;

    private void configureBindings() {
        if (Config.ROBOT_TYPE == RobotType.SETUP_SWERVE_TUNING) {
            drive.setDefaultCommand(DriveCommands.viewWheelForwardDirection(drive, controller::getElevatorAxis));
            controller.stopXBtn().onTrue(new InstantCommand(drive::logModuleOffsets));
            return;
        }

        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
            drive,
            Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveYAxis() : () -> -controller.getDriveYAxis(),
            Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveXAxis() : () -> -controller.getDriveXAxis(),
            Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getTurnAxis() : () -> -controller.getTurnAxis(),
            () -> Config.ROBOT_MODE == RobotMode.SIM ? controller.fieldOrientedBtn().getAsBoolean() : controller.fieldOrientedBtn().getAsBoolean()));
        controller.driveAssistBtn().whileTrue(DriveCommands.driveSuperAssistJoystickDrive(
            drive,
            Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveYAxis() : () -> -controller.getDriveYAxis(),
            Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveXAxis() : () -> -controller.getDriveXAxis()));

        elevator.setDefaultCommand(ElevatorCommands.triggerElevatorHeightAndSetpoint(elevator,
            () -> elevator_setpoint_mode,
            () -> controller.getElevatorAxis(),
            () -> controller.getElevatorAxis(),
            () -> controller.getHingeAxis()));
        climb.setDefaultCommand(ClimbCommands.triggerClimbSpeed(climb, () -> controller.getClimbAxis()));
        mailbox.setDefaultCommand(MailboxCommands.triggerMailboxSpeed(mailbox, () -> controller.getMailboxAxis()));

        controller.algaeLow().onTrue(ElevatorCommands.grabAlgae(elevator, () -> ElevatorSetpoint.ALGAE_LOW));
        controller.algaeHigh().onTrue(ElevatorCommands.grabAlgae(elevator, () -> ElevatorSetpoint.TOP));

        controller.flipRobotBtn().onTrue(DriveCommands.flipRobot(
            drive,
            Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveYAxis() : () -> -controller.getDriveYAxis(),
            Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveXAxis() : () -> -controller.getDriveXAxis()));
        controller.flyToCoralStation1().onTrue(DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Right, () -> ReefBranchHeight.L2));
        controller.flyToCoralStation2().onTrue(DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Left, () -> ReefBranchHeight.L2));

        controller.stopXBtn().whileTrue(DriveCommands.pathfindToPose(drive, FieldConstants.RED_REEF_SIDE_2_POSE));

        // Reset gyro / odometry
        final Runnable resetGyro = Config.ROBOT_MODE == RobotMode.SIM ? () -> drive.setPose(
            drive_simulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        controller.resetGyroBtn().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        controller.elevatorSetpointModeBtn().onTrue(Commands.runOnce(() -> {
            elevator_setpoint_mode = !elevator_setpoint_mode;
        }));
        coral_waiting_trigger.debounce(0.1).onTrue(MailboxCommands.feedCoral(mailbox));
    }

    public void intializePathplannerAutoCommands() {
        NamedCommands.registerCommand("waitfor_coral", Commands.waitUntil(mailbox::coralWaiting));
        // NamedCommands.registerCommand("", null);
    }

    public Command getAutonomousCommand() {
        return auto_chooser.get();
    }

    public void resetSimulationField() {
        if (Config.ROBOT_MODE != RobotMode.SIM)
            return;
        drive_simulation.setSimulationWorldPose(new Pose2d(2, 2, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Config.ROBOT_MODE != RobotMode.SIM)
            return;
        Logger.recordOutput("FieldSimulation/ReefBranchesPoses", FieldConstants.REEF_BRANCHES_POSES());
        Logger.recordOutput("FieldSimulation/RobotPosition", drive_simulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
            "FieldSimulation/Coral",
            SimulatedArena.getInstance().getGamePiecesByType("Coral").toArray(new Pose3d[0]));
        Logger.recordOutput(
            "FieldSimulation/Algae",
            SimulatedArena.getInstance().getGamePiecesByType("Algae").toArray(new Pose3d[0]));

        Logger.recordOutput("Drive/ClosestSnapPoint",
            new Pose2d(
                drive_simulation.getSimulatedDriveTrainPose().getTranslation(),
                new Rotation2d(DriveCommands.closestRotationSnapPoint(drive_simulation.getSimulatedDriveTrainPose()
                    .getRotation()))));
        Logger.recordOutput("FieldSimulation/OdometryToSimulatedTranslationError",
            Units.metersToInches(drive
                .getPose()
                .getTranslation()
                .getDistance(drive_simulation
                    .getSimulatedDriveTrainPose()
                    .getTranslation())));
        Logger.recordOutput("FieldSimulation/OdometryToSimulatedRotationError", Math.abs(drive.getRotation().minus(drive_simulation.getSimulatedDriveTrainPose().getRotation()).getDegrees()));

        Optional<ReefscapeReefSimulation> reefscape_reef_simulation = ReefscapeReefSimulation.getInstance();
        if (reefscape_reef_simulation.isPresent()) {
            int corals_l2 = 0;
            int corals_l3 = 0;
            for (int i = 0; i < 12; i++) {
                corals_l2 += reefscape_reef_simulation.get().getBranches(Config.getRobotAlliance())[i][1];
                corals_l3 += reefscape_reef_simulation.get().getBranches(Config.getRobotAlliance())[i][2];
            }
            Logger.recordOutput("FieldSimulation/L2CoralScored", corals_l2);
            Logger.recordOutput("FieldSimulation/L3CoralScored", corals_l3);
            Logger.recordOutput("FieldSimulation/L2CoralScoredPoints", corals_l2 * 3);
            Logger.recordOutput("FieldSimulation/L3CoralScoredPoints", corals_l3 * 4);
            Logger.recordOutput("FieldSimulation/CoralTotalScoredPoints", (corals_l2 * 3) + (corals_l3 * 4));

        }
    }
}
