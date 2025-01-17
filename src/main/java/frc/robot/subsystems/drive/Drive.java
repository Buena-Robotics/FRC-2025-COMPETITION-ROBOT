package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Config;
import frc.robot.Config.RobotMode;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.sim.COTS;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// Swerve Mk4 L2 Ratio
// https://www.swervedrivespecialties.com/products/mk4-swerve-module
// TODO: Make Sweve Module on coast when disabled
public class Drive extends SubsystemBase {
    public static final double DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / Drive.DRIVE_MOTOR_REDUCTION; // Rotor Rotations -> Wheel Radians
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 / Drive.DRIVE_MOTOR_REDUCTION; // Rotor RPM -> Wheel Rad/Sec

    public static final boolean TURN_INVERTED = false;
    public static final int TURN_MOTOR_CURRENT_LIMIT = 20;
    public static final boolean TURN_ENCODER_INVERTED = false;
    public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians
    public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 50;
    public static final double DRIVE_MOTOR_REDUCTION = 6.75; // MAXSwerve with 14 pinion teeth 22 spur teeth

    public static final double TURN_MOTOR_REDUCTION = 12.8;

    public static final DCMotor DRIVE_GEARBOX = DCMotor.getNeoVortex(1);
    public static final DCMotor TURN_GEARBOX = DCMotor.getNeoVortex(1);

    public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(1.5);
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.2;
    public static final double TRACK_WIDTH = Units.inchesToMeters(29);
    public static final double WHEEL_BASE = Units.inchesToMeters(29);
    public static final double DRIVE_BASE_RADIUS = Math.hypot(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0);
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[] {
            new Translation2d(TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
            new Translation2d(TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0),
            new Translation2d(-TRACK_WIDTH / 2.0, -WHEEL_BASE / 2.0),
            new Translation2d(-TRACK_WIDTH / 2.0, WHEEL_BASE / 2.0)
    };

    public static final double ROBOT_MASS_KG = 45.3592;

    // https://choreo.autos/usage/estimating-moi/
    // You can use SysId to measure by spinning the robot in place and by driving
    // the robot in a straight line.
    // ROBOT_MOI = ROBOT_MASS_KG * (TRACK_WIDTH/2.0) * (KA_ANGULAR / KA_LINEAR);
    // ACURATE
    // ROBOT_MOI = (1/12.0) * ROBOT_MASS_KG * ((TRACK_WIDTH * TRACK_WIDTH) +
    // (WHEEL_BASE * WHEEL_BASE)); SIMPLIFIED
    // ROBOT_MOI = ROBOT_MASS_KG * (TRACK_WIDTH/2.0) * (KA_ANGULAR / KA_LINEAR);
    // SIMPLIFIED + SUBSYSTEM CONCENTRATIONS
    public static final double ROBOT_MOI = 6.883;
    public static final double WHEEL_COF = 1.2;

    public static final SwerveModuleSimulationConfig MARK4 = COTS.ofMark4(TURN_GEARBOX, DRIVE_GEARBOX, WHEEL_COF, 2);

    public static final RobotConfig PP_CONFIG = new RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        new ModuleConfig(
            WHEEL_RADIUS_METERS,
            MAX_SPEED_METERS_PER_SECOND,
            WHEEL_COF,
            DRIVE_GEARBOX.withReduction(DRIVE_MOTOR_REDUCTION),
            DRIVE_MOTOR_CURRENT_LIMIT,
            1),
        MODULE_TRANSLATIONS);

    public static final DriveTrainSimulationConfig MAPLE_SIM_CONFIG = DriveTrainSimulationConfig.Default()
        .withCustomModuleTranslations(MODULE_TRANSLATIONS)
        .withRobotMass(Kilogram.of(ROBOT_MASS_KG))
        .withGyro(COTS.ofNav2X())
        .withSwerveModule(MARK4);

    public static final double ODOMETRY_FREQUENCY_HERTZ = 100.0; // Hz
    public static final Lock odometry_lock = new ReentrantLock();

    private final GyroIO gryo_io;
    private final GyroIOInputsAutoLogged gyro_inputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sys_id;
    private final Alert gyro_disconnect_alert = new Alert("Disconnected gyro, using kinematics as fallback.",
        AlertType.kError);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULE_TRANSLATIONS);
    private Rotation2d raw_gyro_rotation = new Rotation2d();
    private SwerveModulePosition[] last_module_positions = // For delta tracking
        new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
        };
    private SwerveDrivePoseEstimator pose_estimator = new SwerveDrivePoseEstimator(
        kinematics, raw_gyro_rotation, last_module_positions, new Pose2d(3, 3, new Rotation2d()));

    public Drive(GyroIO gyro_io, ModuleIO fl_module, ModuleIO fr_module, ModuleIO bl_module, ModuleIO br_module) {
        this.gryo_io = gyro_io;
        this.modules[0] = new Module(fl_module, 0);
        this.modules[1] = new Module(fr_module, 1);
        this.modules[2] = new Module(bl_module, 2);
        this.modules[3] = new Module(br_module, 3);

        // Usage reporting for swerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        SparkOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
            this::getPose,
            this::setPose,
            this::getChassisSpeeds,
            this::runVelocity,
            new PPHolonomicDriveController(new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
            PP_CONFIG,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            Logger.recordOutput("Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

        // Configure SysId
        sys_id = new SysIdRoutine(
            new SysIdRoutine.Config(null, null, null, (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override public void periodic() {
        odometry_lock.lock(); // Prevents odometry updates while reading data
        gryo_io.updateInputs(gyro_inputs);
        Logger.processInputs("Drive/Gyro", gyro_inputs);
        for (Module module : modules) {
            module.periodic();
        }
        odometry_lock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (Module module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sample_timestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sample_count = sample_timestamps.length;
        for (int i = 0; i < sample_count; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] module_positions = new SwerveModulePosition[4];
            SwerveModulePosition[] module_deltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                module_positions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                module_deltas[moduleIndex] = new SwerveModulePosition(
                    module_positions[moduleIndex].distanceMeters - last_module_positions[moduleIndex].distanceMeters,
                    module_positions[moduleIndex].angle);
                last_module_positions[moduleIndex] = module_positions[moduleIndex];
            }

            // Update gyro angle
            if (gyro_inputs.connected) {
                // Use the real gyro angle
                raw_gyro_rotation = gyro_inputs.odometry_yaw_positions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(module_deltas);
                raw_gyro_rotation = raw_gyro_rotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            pose_estimator.updateWithTime(sample_timestamps[i], raw_gyro_rotation, module_positions);
        }

        // Update gyro alert
        gyro_disconnect_alert.set(!gyro_inputs.connected && Config.ROBOT_MODE != RobotMode.SIM);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds
     *            Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discrete_speeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpoint_states = kinematics.toSwerveModuleStates(discrete_speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpoint_states, MAX_SPEED_METERS_PER_SECOND);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpoint_states);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discrete_speeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpoint_states[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpoint_states);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement.
     * The modules will return to their normal orientations the next time a nonzero
     * velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = MODULE_TRANSLATIONS[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sys_id.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sys_id.dynamic(direction));
    }

    /**
     * Returns the module states (turn angles and drive velocities) for all of the
     * modules.
     */
    @AutoLogOutput(key = "SwerveStates/Measured") private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /**
     * Returns the module positions (turn angles and drive positions) for all of the
     * modules.
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured") private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rad/sec. */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot") public Pose2d getPose() {
        return pose_estimator.getEstimatedPosition();
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        pose_estimator.resetPosition(raw_gyro_rotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
        Pose2d vision_robot_pose_meters, double timestamp_seconds, Matrix<N3, N1> vision_measurement_std_devs) {
        pose_estimator.addVisionMeasurement(vision_robot_pose_meters, timestamp_seconds, vision_measurement_std_devs);
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return MAX_SPEED_METERS_PER_SECOND;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return MAX_SPEED_METERS_PER_SECOND / DRIVE_BASE_RADIUS;
    }
}
