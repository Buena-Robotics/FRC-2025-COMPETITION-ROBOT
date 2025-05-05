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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
import frc.robot.commands.HingeCommands;
import frc.robot.commands.MailboxCommands;
import frc.robot.controller.CommandControllerIO;
import frc.robot.controller.SaitekControllerIO;
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
import frc.robot.subsystems.hinge.Hinge;
import frc.robot.subsystems.hinge.HingeIO;
import frc.robot.subsystems.hinge.HingeIOReal;
import frc.robot.subsystems.hinge.HingeIOSim;
import frc.robot.subsystems.hinge.Hinge.HingeSetpoint;
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
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Utils;

public class RobotContainer {
    // Subsystems
    @SuppressWarnings("unused")
    private final Vision vision;
    private final Drive drive;
    private final SwerveDriveSimulation drive_simulation = Config.ROBOT_MODE == RobotMode.SIM ?
        new SwerveDriveSimulation(Drive.MAPLE_SIM_CONFIG, Utils.initialRobotPose()) :
        null;
    private final Elevator elevator;
    private final Hinge hinge;
    private final Climb climb;
    private final Mailbox mailbox;
    private final Trigger coral_waiting_trigger;
    private final Trigger likely_has_coral_trigger;
    private final Trigger likely_doesnt_has_coral_trigger;
    private final Trigger good_shot_trigger;

    // Controller
    private final CommandControllerIO controller = Config.ROBOT_MODE == RobotMode.SIM ? new SaitekControllerIO(0) : new SaitekControllerIO(0);

    // Dashboard inputs
    private final LoggedTunableNumber auto_delay = new LoggedTunableNumber("AutoDelay/", 0.0);
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
                    drive::addVisionMeasurement, drive, () -> false,
                    new VisionIOPhoton(Cameras.cameras[0]),
                    new VisionIOPhoton(Cameras.cameras[1]),
                    new VisionIOPhoton(Cameras.cameras[2]),
                    new VisionIOPhoton(Cameras.cameras[3]));

                this.elevator = new Elevator(new ElevatorIOReal() {}, drive::getPose);
                this.hinge = new Hinge(new HingeIOReal() {});
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
                this.vision = new Vision(drive::addVisionMeasurement, drive, () -> vision_force_single_tag_mode,
                    new VisionIOPhotonSim(Cameras.cameras[0],
                        drive_simulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonSim(Cameras.cameras[1],
                        drive_simulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonSim(Cameras.cameras[2],
                        drive_simulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonSim(Cameras.cameras[3],
                        drive_simulation::getSimulatedDriveTrainPose));

                this.elevator = new Elevator(new ElevatorIOSim(), drive::getPose);
                this.hinge = new Hinge(new HingeIOSim());

                this.climb = new Climb(new ClimbIOSim());
                this.mailbox = new Mailbox(new MailboxIOSim(drive_simulation, elevator), elevator, drive::getPose);
                break;
            default:
                // Replayed robot, disable IO implementations
                this.drive = new Drive(new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                this.vision = new Vision(drive::addVisionMeasurement, drive, () -> vision_force_single_tag_mode, new VisionIO() {}, new VisionIO() {}, new VisionIO() {});
                this.elevator = new Elevator(new ElevatorIO() {}, drive::getPose);
                this.hinge = new Hinge(new HingeIO() {});
                this.climb = new Climb(new ClimbIO() {});
                this.mailbox = new Mailbox(new MailboxIO() {}, elevator, drive::getPose);
                break;
        }

        this.coral_waiting_trigger = new Trigger(mailbox::coralWaiting);
        this.likely_has_coral_trigger = new Trigger(mailbox::likelyHasCoral);
        this.likely_doesnt_has_coral_trigger = new Trigger(() -> !mailbox.likelyHasCoral());
        this.good_shot_trigger = new Trigger(mailbox::goodShot);

        intializePathplannerAutoCommands();

        // Set up auto routines
        auto_chooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
        auto_chooser.addOption("Leave Community", AutoCommands.leaveCommunity(drive));
        auto_chooser.addOption("BLUE - LEFT - SINGLE", AutoCommands.singleCoralLeftBlue(drive, elevator, mailbox));
        auto_chooser.addOption("BLUE - RIGHT - SINGLE", AutoCommands.singleCoralRightBlue(drive, elevator, mailbox));
        auto_chooser.addOption("RED - LEFT - SINGLE", AutoCommands.singleCoralLeftRed(drive, elevator, mailbox));
        auto_chooser.addOption("RED - RIGHT - SINGLE", AutoCommands.singleCoralRightRed(drive, elevator, mailbox));
        // auto_chooser.addOption("Single Coral", AutoCommands.singleCoral(drive,
        // elevator));

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

    private boolean vision_force_single_tag_mode = false;
    private boolean elevator_setpoint_mode = false;
    private boolean field_oriented_mode = Config.ROBOT_MODE == RobotMode.SIM || Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL ? false : true;

    private void setTeleopDefaultCommands() {
        if (Config.ROBOT_TYPE == RobotType.ROBOT_2025_SCHOOL) {
            drive.setDefaultCommand(DriveCommands.joystickDrive(
                drive,
                Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveYAxis() : () -> -controller.getDriveYAxis(),
                Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveXAxis() : () -> -controller.getDriveXAxis(),
                Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getTurnAxis() : () -> -controller.getTurnAxis(),
                () -> field_oriented_mode));
        } else {
            drive.setDefaultCommand(DriveCommands.driveSuperAssistJoystickDrive(
                drive,
                mailbox,
                Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveYAxis() : () -> -controller.getDriveYAxis(),
                Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveXAxis() : () -> -controller.getDriveXAxis()));
        }
        elevator.setDefaultCommand(ElevatorCommands.triggerElevatorHeightAndSetpoint(elevator,
            () -> elevator_setpoint_mode,
            () -> controller.getElevatorAxis(),
            () -> controller.getElevatorAxis()));
        // hinge.setDefaultCommand(HingeCommands.triggerHingeAngle(hinge, () ->
        // controller.getHingeAxis()));
        climb.setDefaultCommand(ClimbCommands.triggerClimbSpeed(climb, () -> controller.getClimbAxis()));
        mailbox.setDefaultCommand(MailboxCommands.triggerMailboxSpeed(mailbox, () -> controller.getMailboxAxis()));
    }

    private void configureBindings() {
        if (Config.ROBOT_TYPE == RobotType.SETUP_SWERVE_TUNING) {
            drive.setDefaultCommand(DriveCommands.viewWheelForwardDirection(drive, controller::getElevatorAxis));
            // controller.stopXBtn().onTrue(new InstantCommand(drive::logModuleOffsets));
            return;
        }

        // Default command, normal field-relative drive
        setTeleopDefaultCommands();

        controller.modeSemiAuto().whileTrue(DriveCommands.driveSuperAssistAlgaeJoystickDrive(
            drive,
            elevator,
            hinge,
            Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveYAxis() : () -> -controller.getDriveYAxis(),
            Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveXAxis() : () -> -controller.getDriveXAxis()));
        if (Config.ROBOT_TYPE != RobotType.ROBOT_2025_SCHOOL) {
            controller.modeTeleop().whileTrue(DriveCommands.joystickDrive(
                drive,
                Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveYAxis() : () -> -controller.getDriveYAxis(),
                Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveXAxis() : () -> -controller.getDriveXAxis(),
                Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getTurnAxis() : () -> -controller.getTurnAxis(),
                () -> field_oriented_mode));
        }

        // controller.modeTeleop().onTrue(Commands.runOnce(this::setTeleopDefaultCommands));
        // controller.modeSemiAuto().onTrue(Commands.runOnce(this::setSemiAutoDefaultCommands));
        // controller.modeAuto().onTrue(Commands.runOnce(this::setAutoDefaultCommands));

        // controller.modeSemiAuto().onTrue(Commands.runOnce(this::setSemiAutoDefaultCommands));
        // controller.modeAuto().onTrue(Commands.runOnce(this::setAutoDefaultCommands));

        controller.flyToCoralStationLeft().onTrue(Commands.runOnce(() -> {
            vision_force_single_tag_mode = !vision_force_single_tag_mode;
        }));
        controller.fieldOrientedBtn().onTrue(Commands.runOnce(() -> {
            field_oriented_mode = !field_oriented_mode;
        }));
        controller.elevatorSetpointModeBtn().onTrue(Commands.runOnce(() -> {
            elevator_setpoint_mode = !elevator_setpoint_mode;
        }));

        // good_shot_trigger.and(likely_has_coral_trigger).and(new Trigger(() ->
        // vision_force_single_tag_mode)).debounce(0.1)
        // .whileTrue(
        // Commands.deadline(MailboxCommands.triggerMailboxSpeed(mailbox, () -> -1.0))
        // );
        // likely_doesnt_has_coral_trigger.and(controller.modeAuto()).whileTrue(ElevatorCommands.triggerElevatorSetpoint(elevator,
        // ElevatorSetpoint.CORAL_STATION));

        controller.fullClimb().onTrue(ClimbCommands.fullClimb(climb));

        // controller.algaeLow().onTrue(ElevatorCommands.grabAlgae(elevator, () ->
        // ElevatorSetpoint.ALGAE_LOW));
        // controller.algaeHigh().onTrue(ElevatorCommands.grabAlgae(elevator, () ->
        // ElevatorSetpoint.TOP));
        controller.algaeLow().onTrue(Commands.runOnce(() -> {
            hinge.runHingeSetpoint(HingeSetpoint.ALGAE.getValue());
        }, hinge));
        controller.algaeHigh().onTrue(Commands.runOnce(() -> {
            hinge.runHingeSetpoint(HingeSetpoint.TOP.getValue());
        }, hinge));
        controller.algaeRelease().onTrue(HingeCommands.releaseAlgae(hinge));

        // controller.flipRobotBtn().onTrue(DriveCommands.flipRobot(
        // drive,
        // Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveYAxis() : ()
        // -> -controller.getDriveYAxis(),
        // Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveXAxis() : ()
        // -> -controller.getDriveXAxis()));

        controller.flipRobotBtn().onTrue(DriveCommands.driveAssistJoystickDrive(
            drive,
            Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveYAxis() : () -> -controller.getDriveYAxis(),
            Config.ROBOT_MODE == RobotMode.SIM ? () -> -controller.getDriveXAxis() : () -> -controller.getDriveXAxis()));

        controller.flyToClosestReefLeftL2().onTrue(DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Left, () -> ReefBranchHeight.L2));
        controller.flyToClosestReefLeftL3().onTrue(DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Left, () -> ReefBranchHeight.L3));
        controller.flyToClosestReefRightL2().onTrue(DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Right, () -> ReefBranchHeight.L2));
        controller.flyToClosestReefRightL3().onTrue(DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Right, () -> ReefBranchHeight.L3));

        Trigger can_run_coral_feed = new Trigger(() -> DriverStation.isTeleopEnabled());

        coral_waiting_trigger.and(can_run_coral_feed).debounce(0.1).onTrue(MailboxCommands.triggerMailboxSpeed(mailbox, () -> -0.2).until(() -> coral_waiting_trigger.getAsBoolean() == false));
        coral_waiting_trigger.and(can_run_coral_feed).debounce(0.1).onFalse(MailboxCommands.feedCoral(mailbox));

        controller.disableBologna().onTrue(Commands.runOnce(() -> {}, drive));
    }

    public void intializePathplannerAutoCommands() {
        // NamedCommands.registerCommand("algae_low",
        // ElevatorCommands.grabAlgae(elevator, () -> ElevatorSetpoint.ALGAE_LOW));
        // NamedCommands.registerCommand("algae_high",
        // ElevatorCommands.grabAlgae(elevator, () -> ElevatorSetpoint.TOP));
        // NamedCommands.registerCommand("algae_release",
        // ElevatorCommands.releaseAlgae(elevator));

        // NamedCommands.registerCommand("elevator_L3",
        // ElevatorCommands.triggerElevatorSetpoint(elevator, ElevatorSetpoint.L3));
        NamedCommands.registerCommand("elevator_L3", DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Right, () -> ReefBranchHeight.L2).andThen(MailboxCommands.triggerMailboxSpeed(mailbox, () -> -1).withTimeout(2.0)));
        // NamedCommands.registerCommand("algae_low", new WaitCommand(1));
        // NamedCommands.registerCommand("algae_high", new WaitCommand(2));
        // NamedCommands.registerCommand("algae_release", new WaitCommand(1));
        // NamedCommands.registerCommand("coral_3r",
        // DriveCommands.alignToClosestBranch(drive, elevator, ReefBranchSide.Right, ()
        // -> ReefBranchHeight.L3));
        // NamedCommands.registerCommand("waitfor_coral",
        // Commands.waitUntil(mailbox::coralWaiting));

        // ElevatorCommands.grabAlgae(elevator, () -> ElevatorSetpoint.ALGAE_LOW)
        // NamedCommands.registerCommand("", null);
    }

    public Command getAutonomousCommand() {
        CommandScheduler.getInstance().clearComposedCommands();
        return new WaitCommand(auto_delay.get()).andThen(
            auto_chooser.get().andThen(
                Commands.sequence(
                    MailboxCommands.triggerMailboxSpeed(mailbox, () -> -0.1).until(() -> coral_waiting_trigger.getAsBoolean() == false),
                    MailboxCommands.feedCoral(mailbox))));
    }

    public void resetSimulationField() {
        if (Config.ROBOT_MODE != RobotMode.SIM)
            return;
        drive_simulation.setSimulationWorldPose(Utils.initialRobotPose());
        // SimulatedArena.getInstance().resetFieldForAuto();
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

    public void logControlMode() {
        Logger.recordOutput("Control/VisionForceSingleTag", vision_force_single_tag_mode);
        Logger.recordOutput("Control/FieldOrientedMode", field_oriented_mode);
        Logger.recordOutput("Control/ElevatorSetpointMode", elevator_setpoint_mode);
        Logger.recordOutput("Control/Mode",
            controller.modeTeleop().getAsBoolean() ? "Teleop" :
                controller.modeSemiAuto().getAsBoolean() ? "SemiTeleop" :
                    controller.modeAuto().getAsBoolean() ? "Auto" : "UNKNOWN_MODE");
    }
}
