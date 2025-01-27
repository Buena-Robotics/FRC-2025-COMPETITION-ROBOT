package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.util.SparkUtil;

public class ElevatorIOReal implements ElevatorIO {
    private static final int LIFT_MOTOR_CAN_ID = 13;
    private static final double LIFT_P = 0.03;
    private static final double LIFT_D = 0.0;
    private static final int LIFT_MOTOR_CURRENT_LIMIT = 10;
    private static final double LIFT_ENCODER_POSITION_FACTOR = 0.2663130456; // 0.3141592654
    private static final double LIFT_ENCODER_VELOCITY_FACTOR = LIFT_ENCODER_POSITION_FACTOR / 60.0;

    private static final SparkMaxConfig DEFAULT_LIFT_SPARK_CONFIG = defaultLiftSparkConfig();

    private final SparkMax lift_motor;
    private final RelativeEncoder lift_encoder;
    private final SparkClosedLoopController lift_controller;
    private final Debouncer lift_connected_debounce = new Debouncer(0.5);
    private final Debouncer lift_fall_reset_debounce = new Debouncer(Elevator.ELEVATOR_COAST_FALL_TIME_SECONDS);

    private double lift_setpoint_position_inches = 0.0;
    private boolean lift_brake_mode = true;

    public ElevatorIOReal() {
        this.lift_motor = new SparkMax(LIFT_MOTOR_CAN_ID, MotorType.kBrushless);
        this.lift_encoder = this.lift_motor.getEncoder();
        this.lift_controller = this.lift_motor.getClosedLoopController();

        SparkUtil.configureSparkMax(this.lift_motor, DEFAULT_LIFT_SPARK_CONFIG);
        SparkUtil.setPosition(this.lift_motor, this.lift_encoder, 0.0);
    }

    @Override public void updateInputs(final ElevatorIOInputs inputs) {
        SparkUtil.spark_sticky_fault = false;
        inputs.lift_setpoint_position_inches = lift_setpoint_position_inches;
        SparkUtil.ifOk(lift_motor, lift_encoder::getPosition, (value) -> inputs.lift_position_inches = value);
        SparkUtil.ifOk(lift_motor, lift_encoder::getVelocity, (value) -> inputs.lift_velocity_inches_per_second = value);
        SparkUtil.ifOk(lift_motor, new DoubleSupplier[] { lift_motor::getAppliedOutput, lift_motor::getBusVoltage }, (values) -> inputs.lift_applied_volts = values[0] * values[1]);
        SparkUtil.ifOk(lift_motor, lift_motor::getOutputCurrent, (value) -> inputs.lift_current_amps = value);
        inputs.lift_connected = lift_connected_debounce.calculate(!SparkUtil.spark_sticky_fault);
        if (lift_fall_reset_debounce.calculate(!lift_brake_mode)) {
            // zeroLiftPosition();
        }
    }

    @Override public void setLiftPosition(double lift_setpoint_position_inches) {
        // if (lift_setpoint_position_inches < LIFT_CLAMP_MIN_POSITION)
        // lift_setpoint_position_inches = LIFT_CLAMP_MIN_POSITION;
        // if (lift_setpoint_position_inches > Elevator.ELEVATOR_MAX_HEIGHT_INCHES)
        // lift_setpoint_position_inches = Elevator.ELEVATOR_MAX_HEIGHT_INCHES;
        this.lift_setpoint_position_inches = lift_setpoint_position_inches;
        lift_controller.setReference(lift_setpoint_position_inches, ControlType.kPosition);
    }

    @Override public void setLiftBrakeMode(final boolean brake_mode) {
        if (lift_brake_mode == brake_mode)
            return;
        lift_brake_mode = brake_mode;
        DEFAULT_LIFT_SPARK_CONFIG.idleMode(brake_mode ? IdleMode.kBrake : IdleMode.kCoast);
        SparkUtil.configureSparkMax(lift_motor, DEFAULT_LIFT_SPARK_CONFIG);
    }

    @Override public void zeroLiftPosition() {
        SparkUtil.setPosition(lift_motor, lift_encoder, 0.0);
    }

    private static SparkMaxConfig defaultLiftSparkConfig() {
        final SparkMaxConfig lift_config = new SparkMaxConfig();
        // lift_config.softLimit
        // .forwardSoftLimit(Elevator.ELEVATOR_MAX_HEIGHT_INCHES)
        // .forwardSoftLimitEnabled(true)
        // .reverseSoftLimit(0.0)
        // .reverseSoftLimitEnabled(true);
        SparkUtil.setSparkBaseConfig(lift_config, LIFT_MOTOR_CURRENT_LIMIT);
        SparkUtil.setSparkEncoderConfig(lift_config.encoder, LIFT_ENCODER_POSITION_FACTOR, LIFT_ENCODER_VELOCITY_FACTOR);
        SparkUtil.setSparkSignalsConfig(lift_config.signals, 20);
        lift_config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(LIFT_P, 0.0, LIFT_D, 0.0);
        return lift_config;
    }
}
