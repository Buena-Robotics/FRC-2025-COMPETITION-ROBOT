// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.controller.*;
import frc.robot.Config.RobotMode;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroSim;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.vision.Cameras;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonSim;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
    // Subsystems
    @SuppressWarnings("unused")
    private final Vision vision;
    private final Drive drive;
    private final SwerveDriveSimulation drive_simulation = Config.ROBOT_MODE == RobotMode.SIM ?
        new SwerveDriveSimulation(Drive.MAPLE_SIM_CONFIG, new Pose2d(3, 3, new Rotation2d())) :
        null;
    private final Elevator elevator;

    final Servo silly_servo = new Servo(1);
    final Servo silly_servo2 = new Servo(0);

    // Controller
    private final CommandControllerIO controller = new XboxControllerIO(0);

    // Dashboard inputs
    private final LoggedDashboardChooser<Command> auto_chooser;

    public RobotContainer() {
        switch (Config.ROBOT_MODE) {
            case REAL:
                // Real robot, instantiate hardware IO implementations
                this.drive = new Drive(
                    new GyroIONavX(),
                    new ModuleIOSpark(0),
                    new ModuleIOSpark(1),
                    new ModuleIOSpark(2),
                    new ModuleIOSpark(3));

                this.vision = new Vision(
                    drive::addVisionMeasurement
                // new VisionIOPhoton(Cameras.cameras[0])
                );

                this.elevator = new Elevator(new ElevatorIOReal());
                break;
            case SIM:
                // create a maple-sim swerve drive simulation instance
                // add the simulated drivetrain to the simulation field
                SimulatedArena.getInstance().addDriveTrainSimulation(drive_simulation);

                // Sim robot, instantiate physics sim IO implementations
                this.drive = new Drive(
                    new GyroSim(drive_simulation.getGyroSimulation()),
                    new ModuleIOSim(drive_simulation.getModules()[0]),
                    new ModuleIOSim(drive_simulation.getModules()[1]),
                    new ModuleIOSim(drive_simulation.getModules()[2]),
                    new ModuleIOSim(drive_simulation.getModules()[3]));
                this.vision = new Vision(
                    drive::addVisionMeasurement,
                    new VisionIOPhotonSim(Cameras.cameras[0], drive_simulation::getSimulatedDriveTrainPose),
                    new VisionIOPhotonSim(Cameras.cameras[1], drive_simulation::getSimulatedDriveTrainPose));

                this.elevator = new Elevator(new ElevatorIOSim());
                break;
            default:
                // Replayed robot, disable IO implementations
                this.drive = new Drive(
                    new GyroIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {}, new ModuleIO() {});
                this.vision = new Vision(drive::addVisionMeasurement,
                    new VisionIO() {}, new VisionIO() {});
                this.elevator = new Elevator(new ElevatorIO() {});
                break;
        }

        // Set up auto routines
        auto_chooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

        // Set up SysId routines
        auto_chooser.addOption("Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
        auto_chooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
        auto_chooser.addOption(
            "Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        auto_chooser.addOption(
            "Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        auto_chooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        auto_chooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        configureBindings();
    }

    public void updateServo() {
        silly_servo.set(controller.getTestAxis1());
        silly_servo2.set(controller.getTestAxis2());
    }

    private void configureBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(
            drive,
            Config.ROBOT_MODE == RobotMode.REAL ? () -> controller.getDriveYAxis() : () -> -controller.getDriveYAxis(),
            Config.ROBOT_MODE == RobotMode.REAL ? () -> controller.getDriveXAxis() : () -> -controller.getDriveXAxis(),
            Config.ROBOT_MODE == RobotMode.REAL ? () -> controller.getTurnAxis() : () -> -controller.getTurnAxis(),
            () -> false));

        // Lock to 0° when A button is held
        controller
            .lockGyroBtn()
            .whileTrue(DriveCommands.joystickDriveAtAngle(
                drive,
                () -> controller.getDriveYAxis(),
                () -> controller.getDriveXAxis(),
                () -> new Rotation2d()));

        // Switch to X pattern when X button is pressed
        controller.stopXBtn().onTrue(Commands.runOnce(drive::stopWithX, drive));

        // Reset gyro / odometry
        final Runnable resetGyro = Config.ROBOT_MODE == RobotMode.SIM ? () -> drive.setPose(
            drive_simulation
                .getSimulatedDriveTrainPose()) // reset odometry to actual robot pose during simulation
            : () -> drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d())); // zero gyro
        controller.resetGyroBtn().onTrue(Commands.runOnce(resetGyro, drive).ignoringDisable(true));

        elevator.setDefaultCommand(ElevatorCommands.triggerElevatorHeight(elevator, () -> controller.getElevatorAxis()));
    }

    public Command getAutonomousCommand() {
        return auto_chooser.get();
    }

    public void resetSimulationField() {
        if (Config.ROBOT_MODE != RobotMode.SIM)
            return;
        drive_simulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
    }

    public void displaySimFieldToAdvantageScope() {
        if (Config.ROBOT_MODE != RobotMode.SIM)
            return;
        Logger.recordOutput("FieldSimulation/RobotPosition", drive_simulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
            "FieldSimulation/Coral",
            SimulatedArena.getInstance().getGamePiecesByType("Coral").toArray(new Pose3d[0]));
        Logger.recordOutput(
            "FieldSimulation/Algae",
            SimulatedArena.getInstance().getGamePiecesByType("Algae").toArray(new Pose3d[0]));
    }
}
