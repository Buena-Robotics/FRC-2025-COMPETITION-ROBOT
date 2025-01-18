package frc.robot.subsystems.drive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.util.Printf;
import frc.robot.util.SparkUtil;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class ModuleIOSpark implements ModuleIO {
    // 4.071693, 2.830042 + Math.PI, 5.274043, 1.992770 + Math.PI
    public static double[] ZERO_ROTATIONS = {
            4.071693,
            2.830042 + Math.PI,
            5.274043,
            1.992770 };

    public static final double DRIVE_P = 0.0;
    public static final double DRIVE_D = 0.0;
    public static final double DRIVE_S = 0.0;
    public static final double DRIVE_V = 0.1;

    public static final double TURN_P = 0.12;
    public static final double TURN_D = 0.0;
    public static final double TURN_PID_MIN_INPUT_RADIANS = 0;
    public static final double TURN_PID_MAX_INPUT_RADIANS = 2 * Math.PI;

    // Hardware objects
    private final SparkMax drive_motor;
    private final SparkMax turn_motor;
    private final RelativeEncoder drive_encoder;
    private final RelativeEncoder turn_encoder;
    private final AnalogEncoder turn_absolute_encoder;

    // Closed loop controllers
    private final SparkClosedLoopController drive_controller;
    private final SparkClosedLoopController turn_controller;

    // Queue inputs from odometry thread
    private final Queue<Double> timestamp_queue;
    private final Queue<Double> drive_position_queue;
    private final Queue<Double> turn_position_queue;

    // Connection debouncers
    private final Debouncer drive_connected_debounce = new Debouncer(0.5);
    private final Debouncer turn_connected_debounce = new Debouncer(0.5);

    public ModuleIOSpark(final int module) {
        this.turn_absolute_encoder = new AnalogEncoder(module, 2.0 * Math.PI, ZERO_ROTATIONS[module]);
        this.drive_motor = new SparkMax((module * 2) + 1, MotorType.kBrushless);
        this.turn_motor = new SparkMax((module * 2) + 2, MotorType.kBrushless);
        this.drive_encoder = drive_motor.getEncoder();
        this.turn_encoder = turn_motor.getEncoder();
        this.drive_controller = drive_motor.getClosedLoopController();
        this.turn_controller = turn_motor.getClosedLoopController();

        // Configure drive motor
        final SparkMaxConfig drive_config = new SparkMaxConfig();
        drive_config
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Drive.DRIVE_MOTOR_CURRENT_LIMIT)
            .voltageCompensation(12.0);
        drive_config.encoder
            .positionConversionFactor(Drive.DRIVE_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(Drive.DRIVE_ENCODER_VELOCITY_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        drive_config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(
                DRIVE_P, 0.0,
                DRIVE_D, 0.0);
        drive_config.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / Drive.ODOMETRY_FREQUENCY_HERTZ))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        SparkUtil.tryUntilOk(
            drive_motor,
            5,
            () -> drive_motor.configure(
                drive_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        SparkUtil.tryUntilOk(drive_motor, 5, () -> drive_encoder.setPosition(0.0));

        // Configure turn motor
        final SparkMaxConfig turn_config = new SparkMaxConfig();
        turn_config
            .inverted(Drive.TURN_INVERTED)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Drive.TURN_MOTOR_CURRENT_LIMIT)
            .voltageCompensation(12.0);
        turn_config.encoder
            .positionConversionFactor((1.0 / Drive.TURN_MOTOR_REDUCTION) * Drive.TURN_ENCODER_POSITION_FACTOR)
            .velocityConversionFactor(Drive.TURN_ENCODER_VELOCITY_FACTOR)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        turn_config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(TURN_PID_MIN_INPUT_RADIANS, TURN_PID_MAX_INPUT_RADIANS)
            .pidf(TURN_P, 0.0, TURN_D, 0.0);
        turn_config.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / Drive.ODOMETRY_FREQUENCY_HERTZ))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        SparkUtil.tryUntilOk(
            turn_motor,
            5,
            () -> turn_motor.configure(turn_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
        SparkUtil.tryUntilOk(turn_motor, 5, () -> turn_encoder.setPosition(turn_absolute_encoder.get()));

        Printf.info("MODULE OFFSET (%d): %f", module, turn_absolute_encoder.get());

        // Create odometry queues
        this.timestamp_queue = SparkOdometryThread.getInstance().makeTimestampQueue();
        this.drive_position_queue = SparkOdometryThread.getInstance().registerSignal(drive_motor, drive_encoder::getPosition);
        this.turn_position_queue = SparkOdometryThread.getInstance().registerSignal(turn_motor, turn_encoder::getPosition);
    }

    @Override public void updateInputs(final ModuleIOInputs inputs) {
        // Update drive inputs
        SparkUtil.spark_sticky_fault = false;
        SparkUtil.ifOk(drive_motor, drive_encoder::getPosition, (value) -> inputs.drive_position_radians = value);
        SparkUtil.ifOk(
            drive_motor, drive_encoder::getVelocity, (value) -> inputs.drive_velocity_radians_per_second = value);
        SparkUtil.ifOk(
            drive_motor,
            new DoubleSupplier[] { drive_motor::getAppliedOutput, drive_motor::getBusVoltage },
            (values) -> inputs.drive_applied_volts = values[0] * values[1]);
        SparkUtil.ifOk(drive_motor, drive_motor::getOutputCurrent, (value) -> inputs.drive_current_amps = value);
        inputs.drive_connected = drive_connected_debounce.calculate(!SparkUtil.spark_sticky_fault);

        // Update turn inputs
        inputs.turn_absolute_position = new Rotation2d(turn_absolute_encoder.get());
        SparkUtil.spark_sticky_fault = false;
        SparkUtil.ifOk(
            turn_motor,
            turn_encoder::getPosition,
            (value) -> inputs.turn_position = new Rotation2d(value));
        SparkUtil.ifOk(
            turn_motor, turn_encoder::getVelocity, (value) -> inputs.turn_velocity_radians_per_second = value);
        SparkUtil.ifOk(
            turn_motor,
            new DoubleSupplier[] { turn_motor::getAppliedOutput, turn_motor::getBusVoltage },
            (values) -> inputs.turn_applied_volts = values[0] * values[1]);
        SparkUtil.ifOk(turn_motor, turn_motor::getOutputCurrent, (value) -> inputs.turn_current_amps = value);
        inputs.turn_connected = turn_connected_debounce.calculate(!SparkUtil.spark_sticky_fault);

        // Update odometry inputs
        inputs.odometry_timestamps = timestamp_queue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometry_drive_position_radians = drive_position_queue.stream()
            .mapToDouble((Double value) -> value)
            .toArray();
        inputs.odometry_turn_positions = turn_position_queue.stream()
            .map((Double value) -> new Rotation2d(value))
            .toArray(Rotation2d[]::new);
        this.timestamp_queue.clear();
        this.drive_position_queue.clear();
        this.turn_position_queue.clear();
    }

    @Override public void setDriveOpenLoop(final double output) {
        this.drive_motor.setVoltage(output);
    }

    @Override public void setTurnOpenLoop(final double output) {
        this.turn_motor.setVoltage(output);
    }

    @Override public void setDriveVelocity(final double velocity_radians_per_second) {
        double ffVolts = DRIVE_S * Math.signum(velocity_radians_per_second) + DRIVE_V * velocity_radians_per_second;
        this.drive_controller.setReference(
            velocity_radians_per_second, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override public void setTurnPosition(final Rotation2d rotation) {
        double setpoint = MathUtil.inputModulus(
            rotation.getRadians(), TURN_PID_MIN_INPUT_RADIANS, TURN_PID_MAX_INPUT_RADIANS);
        this.turn_controller.setReference(setpoint, ControlType.kPosition);
    }

    @Override public void setDriveBrakeMode(final boolean brake_mode) {
        // drive_motor.configure(null, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    @Override public void setTurnBrakeMode(final boolean brake_mode) {
        // turn_motor.configure(null, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
