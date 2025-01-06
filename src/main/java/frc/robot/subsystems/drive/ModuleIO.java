package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public abstract class ModuleIO {
    @AutoLog
    public static class ModuleIOInputs {
        public boolean drive_connected = false;
        public double drive_position_radians = 0.0;
        public double drive_velocity_radians_per_second = 0.0;
        public double drive_applied_volts = 0.0;
        public double drive_current_amps = 0.0;

        public boolean turn_connected = false;
        public Rotation2d turn_position = new Rotation2d();
        public double turn_velocity_radians_per_second = 0.0;
        public double turn_applied_volts = 0.0;
        public double turn_current_amps = 0.0;

        public double[] odometry_timestamps = new double[] {};
        public double[] odometry_drive_position_radians = new double[] {};
        public Rotation2d[] odometry_turn_positions = new Rotation2d[] {};
    }

    public static final double DRIVE_ENCODER_POSITION_FACTOR = 2 * Math.PI / Drive.DRIVE_MOTOR_REDUCTION; // Rotor Rotations -> Wheel Radians
    public static final double DRIVE_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0 / Drive.DRIVE_MOTOR_REDUCTION; // Rotor RPM -> Wheel Rad/Sec

    public static final boolean TURN_INVERTED = false;
    public static final int TURN_MOTOR_CURRENT_LIMIT = 20;
    public static final double TURN_MOTOR_REDUCTION = 9424.0 / 203.0;
    public static final DCMotor TURN_GEARBOX = DCMotor.getNeo550(1);
    public static final boolean TURN_ENCODER_INVERTED = true;
    public static final double TURN_ENCODER_POSITION_FACTOR = 2 * Math.PI; // Rotations -> Radians
    public static final double TURN_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;

    private final Alert drive_disconnect_alert;
    private final Alert turn_disconnected_alert;
    private SwerveModulePosition[] odometry_positions = new SwerveModulePosition[] {};

    public ModuleIO(final int index) {
        this.index = index;
        drive_disconnect_alert = new Alert(
                "Disconnected drive motor on module " + Integer.toString(index) + ".",
                AlertType.kError);
        turn_disconnected_alert = new Alert(
                "Disconnected turn motor on module " + Integer.toString(index) + ".", AlertType.kError);
    }

    public void periodic() {
        updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

        // Calculate positions for odometry
        int sample_count = inputs.odometry_timestamps.length; // All signals are sampled together
        odometry_positions = new SwerveModulePosition[sample_count];
        for (int i = 0; i < sample_count; i++) {
            double position_meters = inputs.odometry_drive_position_radians[i] * Drive.WHEEL_RADIUS_METERS;
            Rotation2d angle = inputs.odometry_turn_positions[i];
            odometry_positions[i] = new SwerveModulePosition(position_meters, angle);
        }

        // Update alerts
        drive_disconnect_alert.set(!inputs.drive_connected);
        turn_disconnected_alert.set(!inputs.turn_connected);
    }

    /**
     * Runs the module with the specified setpoint state. Mutates the state to
     * optimize it.
     */
    public void runSetpoint(SwerveModuleState state) {
        // Optimize velocity setpoint
        state.optimize(getAngle());
        state.cosineScale(inputs.turn_position);

        // Apply setpoints
        setDriveVelocity(state.speedMetersPerSecond / Drive.WHEEL_RADIUS_METERS);
        setTurnPosition(state.angle);
    }

    /**
     * Runs the module with the specified output while controlling to zero degrees.
     */
    public void runCharacterization(double output) {
        setDriveOpenLoop(output);
        setTurnPosition(new Rotation2d());
    }

    /** Disables all outputs to motors. */
    public void stop() {
        setDriveOpenLoop(0.0);
        setTurnOpenLoop(0.0);
    }

    /** Returns the current turn angle of the module. */
    public Rotation2d getAngle() {
        return inputs.turn_position;
    }

    /** Returns the current drive position of the module in meters. */
    public double getPositionMeters() {
        return inputs.drive_position_radians * Drive.WHEEL_RADIUS_METERS;
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.drive_velocity_radians_per_second * Drive.WHEEL_RADIUS_METERS;
    }

    /** Returns the module position (turn angle and drive position). */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    /** Returns the module state (turn angle and drive velocity). */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    /** Returns the module positions received this cycle. */
    public SwerveModulePosition[] getOdometryPositions() {
        return odometry_positions;
    }

    /** Returns the timestamps of the samples received this cycle. */
    public double[] getOdometryTimestamps() {
        return inputs.odometry_timestamps;
    }

    /** Returns the module position in radians. */
    public double getWheelRadiusCharacterizationPosition() {
        return inputs.drive_position_radians;
    }

    /** Returns the module velocity in rad/sec. */
    public double getFFCharacterizationVelocity() {
        return inputs.drive_velocity_radians_per_second;
    }

    /** Updates the set of loggable inputs. */
    public abstract void updateInputs(ModuleIOInputs inputs);

    /** Run the drive motor at the specified open loop value. */
    public abstract void setDriveOpenLoop(double output);

    /** Run the turn motor at the specified open loop value. */
    public abstract void setTurnOpenLoop(double output);

    /** Run the drive motor at the specified velocity. */
    public abstract void setDriveVelocity(double velocityRadPerSec);

    /** Run the turn motor to the specified rotation. */
    public abstract void setTurnPosition(Rotation2d rotation);
}