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
import frc.robot.subsystems.drive.Drive;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linear_magnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linear_direction = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linear_magnitude = linear_magnitude * linear_magnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linear_direction)
                .transformBy(new Transform2d(linear_magnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    /** Field relative drive command using two joysticks (controlling linear and angular velocities). */
    public static Command joystickDrive(
            Drive drive, DoubleSupplier x_supplier, DoubleSupplier y_supplier, DoubleSupplier omega_supplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linear_velocity =
                            getLinearVelocityFromJoysticks(x_supplier.getAsDouble(), y_supplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omega_supplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds = new ChassisSpeeds(
                            linear_velocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linear_velocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());
                    boolean is_flipped = DriverStation.getAlliance().isPresent()
                            && DriverStation.getAlliance().get() == Alliance.Red;
                    drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                            speeds,
                            is_flipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
                },
                drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control. Possible use cases
     * include snapping to an angle, aiming at a vision target, or controlling absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive, DoubleSupplier x_supplier, DoubleSupplier y_supplier, Supplier<Rotation2d> rotation_supplier) {

        // Create PID controller
        ProfiledPIDController angle_controller = new ProfiledPIDController(
                ANGLE_KP, 0.0, ANGLE_KD, new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angle_controller.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linear_velocity =
                                    getLinearVelocityFromJoysticks(x_supplier.getAsDouble(), y_supplier.getAsDouble());

                            // Calculate angular speed
                            double omega = angle_controller.calculate(
                                    drive.getRotation().getRadians(),
                                    rotation_supplier.get().getRadians());

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(
                                    linear_velocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                    linear_velocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                    omega);
                            boolean is_flipped = DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                            drive.runVelocity(ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    is_flipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                        },
                        drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angle_controller.reset(drive.getRotation().getRadians()));
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
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
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(() -> {
                            limiter.reset(0.0);
                        }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

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
                                    var rotation = drive.getRotation();
                                    state.gyro_delta += Math.abs(
                                            rotation.minus(state.last_angle).getRadians());
                                    state.last_angle = rotation;
                                })

                                // When cancelled, calculate and print results
                                .finallyDo(() -> {
                                    double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                    double wheel_delta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheel_delta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheel_radius = (state.gyro_delta * Drive.DRIVE_BASE_RADIUS) / wheel_delta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println("********** Wheel Radius Characterization Results **********");
                                    System.out.println("\tWheel Delta: " + formatter.format(wheel_delta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyro_delta) + " radians");
                                    System.out.println("\tWheel Radius: "
                                            + formatter.format(wheel_radius)
                                            + " meters, "
                                            + formatter.format(Units.metersToInches(wheel_radius))
                                            + " inches");
                                })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d last_angle = new Rotation2d();
        double gyro_delta = 0.0;
    }
}
