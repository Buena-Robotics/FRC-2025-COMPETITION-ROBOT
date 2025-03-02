package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Config;
import frc.robot.FieldConstants;
import frc.robot.FieldConstants.ReefBranchHeight;
import frc.robot.FieldConstants.ReefBranchSide;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Printf;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

public class DriveCommands {
    private static final double DEADBAND = 0.10;
    private static final double DRIVE_KP = 1.0;
    private static final double DRIVE_KI = 0.008;
    private static final double DRIVE_KD = 0.2;
    private static final double ANGLE_KP = 3.5;
    private static final double ANGLE_KI = 0.01;
    private static final double ANGLE_KD = 0.3;
    private static final double DRIVE_MAX_ACCELERATION = 1.5;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private final static ProfiledPIDController x_controller = new ProfiledPIDController(
        DRIVE_KP, DRIVE_KI, DRIVE_KD, new TrapezoidProfile.Constraints(Drive.getMaxLinearSpeedMetersPerSec(), DRIVE_MAX_ACCELERATION));
    private final static ProfiledPIDController y_controller = new ProfiledPIDController(
        DRIVE_KP, DRIVE_KI, DRIVE_KD, new TrapezoidProfile.Constraints(Drive.getMaxLinearSpeedMetersPerSec(), DRIVE_MAX_ACCELERATION));
    private final static ProfiledPIDController angle_controller = new ProfiledPIDController(
        ANGLE_KP, ANGLE_KI, ANGLE_KD, new TrapezoidProfile.Constraints(Drive.getMaxAngularSpeedRadPerSec(), ANGLE_MAX_ACCELERATION));

    static {
        x_controller.setIZone(Units.inchesToMeters(2));
        x_controller.setIntegratorRange(0, Units.inchesToMeters(2));
        x_controller.setTolerance(Units.inchesToMeters(0.25));
        y_controller.setIZone(Units.inchesToMeters(2));
        y_controller.setIntegratorRange(0, Units.inchesToMeters(2));
        y_controller.setTolerance(Units.inchesToMeters(0.25));

        angle_controller.enableContinuousInput(-Math.PI, Math.PI);
        angle_controller.setIZone(0.25);
        angle_controller.setIntegratorRange(0, 0.25);
        angle_controller.setTolerance(Math.PI / 180.0);
    }

    private DriveCommands() {}

    private static Rotation2d[] reef_assist_snap_points = {
            new Rotation2d(), // Forward
            new Rotation2d(Math.PI), // Backward
            new Rotation2d(Math.PI / 3.0), // 60 degrees left
            new Rotation2d(-Math.PI / 3.0), // 60 degrees right
            new Rotation2d(2 * Math.PI / 3.0), // 120 degrees left
            new Rotation2d(-2 * Math.PI / 3.0), // 120 degrees right
    };

    private static Rotation2d[] turn_assist_snap_points = {
            new Rotation2d(), // Forward
            new Rotation2d(Math.PI), // Backward
            new Rotation2d(Math.PI / 3.0), // 60 degrees left
            new Rotation2d(-Math.PI / 3.0), // 60 degrees right
            new Rotation2d(2 * Math.PI / 3.0), // 120 degrees left
            new Rotation2d(-2 * Math.PI / 3.0), // 120 degrees right
            Rotation2d.fromDegrees(126).plus(new Rotation2d(Math.PI)), // Left Coral Station
            Rotation2d.fromDegrees(234).plus(new Rotation2d(Math.PI)), // Right Coral Station
    };

    private static void resetControllers(final Drive drive) {
        x_controller.reset(drive.getPose().getX());
        y_controller.reset(drive.getPose().getY());
        angle_controller.reset(drive.getRotation().getRadians(), drive.yawRate());
    }

    public static double closestReefRotationSnapPoint(Rotation2d estimate_radians) {
        int closest_index = 0;
        double closest_distance = Double.MAX_VALUE;
        for (int i = 0; i < reef_assist_snap_points.length; i++) {
            final Rotation2d snap_point = Config.getRobotAlliance().equals(Alliance.Blue) ? reef_assist_snap_points[i] : reef_assist_snap_points[i].plus(new Rotation2d(Math.PI));
            final double distance = Math.abs(estimate_radians.minus(snap_point).getRadians());
            if (distance < closest_distance) {
                closest_distance = distance;
                closest_index = i;
            }
        }
        return reef_assist_snap_points[closest_index].getRadians();
    }

    public static double closestRotationSnapPoint(Rotation2d estimate_radians) {
        int closest_index = 0;
        double closest_distance = Double.MAX_VALUE;
        for (int i = 0; i < turn_assist_snap_points.length; i++) {
            final Rotation2d snap_point = Config.getRobotAlliance().equals(Alliance.Blue) ? turn_assist_snap_points[i] : turn_assist_snap_points[i].plus(new Rotation2d(Math.PI));
            final double distance = Math.abs(estimate_radians.minus(snap_point).getRadians());
            if (distance < closest_distance) {
                closest_distance = distance;
                closest_index = i;
            }
        }
        return turn_assist_snap_points[closest_index].getRadians();
    }

    public static Pose2d getClosestReefPose(final Drive drive) {
        int closest_index = 0;
        double closest_distance = Double.MAX_VALUE;
        Pose2d[] pose_list = FieldConstants.REEF_SIDE_POSES();
        for (int i = 0; i < pose_list.length; i++) {
            double distance = drive.getPose().getTranslation().getDistance(pose_list[i].getTranslation());
            if (distance < closest_distance) {
                closest_distance = distance;
                closest_index = i;
            }
        }
        return pose_list[closest_index];
    }

    private static Translation2d getLinearVelocityFromJoysticks(final double x, final double y) {
        // Apply deadband
        final Rotation2d linear_direction = new Rotation2d(Math.atan2(y, x));
        double linear_magnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);

        // Square magnitude for more precise control
        linear_magnitude = linear_magnitude * linear_magnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linear_direction)
            .transformBy(new Transform2d(linear_magnitude, 0.0, new Rotation2d()))
            .getTranslation();
    }

    private static void runSpeeds(final Drive drive, final double x_in, final double y_in, final double omega_in, final boolean field_oriented) {
        // Get linear velocity
        final Translation2d linear_velocity = getLinearVelocityFromJoysticks(x_in, y_in);

        // Convert to field relative speeds & send command
        final ChassisSpeeds speeds = new ChassisSpeeds(
            linear_velocity.getX() * Drive.getMaxLinearSpeedMetersPerSec(),
            linear_velocity.getY() * Drive.getMaxLinearSpeedMetersPerSec(),
            omega_in * Drive.getMaxAngularSpeedRadPerSec());

        if (field_oriented) {
            final boolean is_flipped = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;
            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                speeds,
                is_flipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
        } else {
            drive.runVelocity(speeds);
        }
    }

    /* Drive only forward wtih joystick to set
     * module abs encoder rotations */
    public static Command joystickForwardOnlyDrive(final Drive drive, final DoubleSupplier x_supplier) {
        return Commands.run(
            () -> {
                runSpeeds(drive, x_supplier.getAsDouble(), 0, 0, false);
            }, drive);
    }

    public static Command flipRobot(final Drive drive, final DoubleSupplier x_supplier, final DoubleSupplier y_supplier, final BooleanSupplier field_oriented_supplier) {
        return Commands.deadline(new WaitCommand(1.5),
            Commands.run(() -> {
                // Calculate angular speed
                final double omega = angle_controller.calculate(drive.getRotation().getRadians());

                runSpeeds(drive, x_supplier.getAsDouble(), y_supplier.getAsDouble(), omega, field_oriented_supplier.getAsBoolean());
            }, drive).until(() -> angle_controller.atGoal()))
            .beforeStarting(() -> {
                resetControllers(drive);
                angle_controller.setGoal(drive.getRotation().minus(new Rotation2d(Math.PI)).getRadians());
            });
    }

    public static Command driveAssistJoystickDrive(final Drive drive, final DoubleSupplier x_supplier, final DoubleSupplier y_supplier, final BooleanSupplier field_oriented_supplier) {
        return Commands.run(() -> {
                // Calculate angular speed
                final double omega = angle_controller.calculate(drive.getRotation().getRadians());

                runSpeeds(drive, x_supplier.getAsDouble(), y_supplier.getAsDouble(), omega, field_oriented_supplier.getAsBoolean());
            }, drive).until(() -> angle_controller.atGoal())
            .beforeStarting(() -> {
                resetControllers(drive);
                angle_controller.setGoal(new Rotation2d(closestRotationSnapPoint(drive.getRotation())).getRadians());
            });
    }

    public static Command driveSuperAssistJoystickDrive(final Drive drive, final DoubleSupplier x_supplier, final DoubleSupplier y_supplier, final BooleanSupplier field_oriented_supplier) {
        return Commands.run(() -> {
            // Get linear velocity
            final Pose2d robot_pose = drive.getPose();
            final double relative_x = robot_pose.getX() - Units.inchesToMeters(144 + 32.75);
            final double relative_y = robot_pose.getY() - Units.inchesToMeters(158.50);

            final double super_assist_robot_rotation = closestReefRotationSnapPoint(
                new Rotation2d(
                    Math.atan2(relative_y, relative_x)));

            // Calculate angular speed
            final double omega = angle_controller.calculate(
                drive.getRotation().minus(new Rotation2d(
                    Math.PI)).getRadians(),
                super_assist_robot_rotation);

            runSpeeds(drive, x_supplier.getAsDouble(), y_supplier.getAsDouble(), omega, field_oriented_supplier.getAsBoolean());
        }, drive)
        .beforeStarting(() -> {
            resetControllers(drive);
        });
    }

    public static Command alignToClosestBranch(final Drive drive, final Supplier<ReefBranchSide> branch_side, final Supplier<ReefBranchHeight> branch_height) {
        return Commands.run(
            () -> {
                final Pose2d robot_pose = drive.getPose();
                final Pose2d closest_reef_pose = getClosestReefPose(drive);
                runSpeeds(
                    drive,
                    x_controller.calculate(robot_pose.getX(), closest_reef_pose.getX()),
                    y_controller.calculate(robot_pose.getY(), closest_reef_pose.getY()),
                    angle_controller.calculate(robot_pose.getRotation().getRadians(), closest_reef_pose.getRotation().getRadians()),
                    true);
            },
            drive)
            .until(() -> x_controller.atGoal() && y_controller.atGoal() && angle_controller.atGoal())
            .beforeStarting(() -> {
                final Pose2d closest_reef_pose = getClosestReefPose(drive);
                resetControllers(drive);
                x_controller.setGoal(closest_reef_pose.getX());
                y_controller.setGoal(closest_reef_pose.getY());
                angle_controller.setGoal(closest_reef_pose.getRotation().getRadians());
            });
    }

    /**
     * Drive command using two joysticks (controlling linear and
     * angular velocities).
     */
    public static Command joystickDrive(final Drive drive, final DoubleSupplier x_supplier, final DoubleSupplier y_supplier, final DoubleSupplier omega_supplier, final BooleanSupplier field_oriented_supplier) {
        return Commands.run(
            () -> {
                double omega = MathUtil.applyDeadband(omega_supplier.getAsDouble(), DEADBAND);

                omega = Math.copySign(omega * omega * omega, omega);
                omega = MathUtil.clamp(omega, -0.6, 0.6);
                runSpeeds(drive, x_supplier.getAsDouble(), y_supplier.getAsDouble(), omega, field_oriented_supplier.getAsBoolean());
            },
            drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for
     * angular control. Possible use cases
     * include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(final Drive drive, final DoubleSupplier x_supplier, final DoubleSupplier y_supplier, final Supplier<Rotation2d> rotation_supplier) {
        // Construct command
        return Commands.run(
            () -> {
                // Calculate angular speed
                double omega = angle_controller.calculate(
                    drive.getRotation().getRadians(),
                    rotation_supplier.get().getRadians());

                runSpeeds(drive, x_supplier.getAsDouble(), y_supplier.getAsDouble(), omega, true);
            },
            drive)
            // Reset PID controller when command starts
            .beforeStarting(() -> angle_controller.reset(drive.getRotation().getRadians()));
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(final Drive drive) {
        List<Double> velocity_samples = new LinkedList<>();
        List<Double> voltage_samples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(() -> {
                velocity_samples.clear();
                voltage_samples.clear();
            }),

            // Allow modules to orient
            Commands.run(
                () -> {
                    drive.runCharacterization(0.0);
                },
                drive)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                () -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    drive.runCharacterization(voltage);
                    velocity_samples.add(drive.getFFCharacterizationVelocity());
                    voltage_samples.add(voltage);
                },
                drive)

                // When cancelled, calculate and print results
                .finallyDo(() -> {
                    int n = velocity_samples.size();
                    double sum_x = 0.0;
                    double sum_y = 0.0;
                    double sum_xy = 0.0;
                    double sum_x2 = 0.0;
                    for (int i = 0; i < n; i++) {
                        sum_x += velocity_samples.get(i);
                        sum_y += voltage_samples.get(i);
                        sum_xy += velocity_samples.get(i) * voltage_samples.get(i);
                        sum_x2 += velocity_samples.get(i) * velocity_samples.get(i);
                    }
                    double ks = (sum_y * sum_x2 - sum_x * sum_xy) / (n * sum_x2 - sum_x * sum_x);
                    double kv = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);

                    NumberFormat formatter = new DecimalFormat("#0.00000");
                    System.out.println("********** Drive FF Characterization Results **********");
                    System.out.println("\tkS: " + formatter.format(ks));
                    System.out.println("\tkV: " + formatter.format(kv));
                }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(final Drive drive) {
        final SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        final WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
            // Drive control sequence
            Commands.sequence(
                // Reset acceleration limiter
                Commands.runOnce(() -> {
                    limiter.reset(0.0);
                }),

                // Turn in place, accelerating up to full speed
                Commands.run(() -> {
                    final double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                }, drive)),

            // Measurement sequence
            Commands.sequence(
                // Wait for modules to fully orient before starting measurement
                Commands.waitSeconds(1.0),

                // Record starting measurement
                Commands.runOnce(() -> {
                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                    state.last_angle = drive.getRotation();
                    state.gyro_delta = 0.0;
                }),

                // Update gyro delta
                Commands.run(() -> {
                    final Rotation2d rotation = drive.getRotation();
                    state.gyro_delta += Math.abs(
                        rotation.minus(state.last_angle).getRadians());
                    state.last_angle = rotation;
                })
                    // When cancelled, calculate and print results
                    .finallyDo(() -> {
                        final double[] positions = drive.getWheelRadiusCharacterizationPositions();
                        double wheel_delta = 0.0;
                        for (int i = 0; i < 4; i++) {
                            wheel_delta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                        }
                        final double wheel_radius = (state.gyro_delta * Drive.DRIVE_BASE_RADIUS) / wheel_delta;

                        final NumberFormat formatter = new DecimalFormat("#0.000");
                        System.out.println("********** Wheel Radius Characterization Results **********");
                        System.out.println("\tWheel Delta: " + formatter.format(wheel_delta) + " radians");
                        System.out.println(
                            "\tGyro Delta: " + formatter.format(state.gyro_delta) + " radians");
                        System.out.println("\tWheel Radius: " + formatter.format(wheel_radius) + " meters, " + formatter.format(Units.metersToInches(wheel_radius)) + " inches");
                    })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d last_angle = new Rotation2d();
        double gyro_delta = 0.0;
    }

    public static Command trueMaxDriveSpeedCharacterization(final Drive drive) {
        return Commands.parallel(
            Commands.run(() -> {
                Printf.info("True Max Drive Speed(m/s): %f");
            }, drive),
            Commands.sequence(Commands.waitSeconds(3.0), Commands.runOnce(() -> {

            })));
    }

    public static Command viewWheelForwardCharacterization(final Drive drive, final DoubleSupplier voltage) {
        return Commands.run(() -> drive.runCharacterization(voltage.getAsDouble() * 2), drive);
    }

    public static Command viewWheelForwardDirection(final Drive drive, final DoubleSupplier voltage) {
        return Commands.run(() -> drive.runForward(voltage.getAsDouble()), drive);
    }

    private static final PathConstraints pathfinding_constraints = new PathConstraints(2.4, 1.25, Units.degreesToRadians(540), Units.degreesToRadians(720));

    public static Command pathfindToPose(final Drive drive, final Pose2d pose) {
        return AutoBuilder.pathfindToPose(pose, pathfinding_constraints, 0.0);
    }

    public static Command pathfindToPoseSupplier(final Drive drive, final Supplier<Pose2d> pose_supplier) {
        return Commands.defer(
            () -> AutoBuilder.pathfindToPose(pose_supplier.get(), pathfinding_constraints),
            Set.of(drive));
    }
}
