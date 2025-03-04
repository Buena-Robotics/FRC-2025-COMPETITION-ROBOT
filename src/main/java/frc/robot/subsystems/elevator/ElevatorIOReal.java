package frc.robot.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
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
    private static final int LIFT_MOTOR_CAN_ID = 9;
    private static final int HINGE_MOTOR_CAN_ID = 14;

    // Constants when lift is empty
    private static final double LIFT_EMPTY_P = 0.4;
    private static final double LIFT_EMPTY_D = 0.025;
    private static final double HINGE_P = 0.1;
    private static final double HINGE_D = 0.005;

    private static final double LIFT_CLAMP_MIN_POSITION = 0.25;

    private static final int LIFT_MOTOR_CURRENT_LIMIT = 10;
    private static final double LIFT_ENCODER_POSITION_FACTOR = 1.0 / 2.7643; // Math.PI * 2 * (1.0/Elevator.LIFT_MOTOR_REDUCTION);
    private static final double LIFT_ENCODER_VELOCITY_FACTOR = LIFT_ENCODER_POSITION_FACTOR / 60.0;
    private static final int HINGE_MOTOR_CURRENT_LIMIT = 18;

    // private static final double HINGE_ABSOLUTE_ENCODER_END_POSITION = 0.0;

    // private static final int

    private static final SparkMaxConfig DEFAULT_LIFT_SPARK_CONFIG = defaultLiftSparkConfig();

    private final SparkMax lift_motor;
    private final RelativeEncoder lift_encoder;
    private final SparkClosedLoopController lift_controller;
    private final Debouncer lift_connected_debounce = new Debouncer(0.5);

    private final SparkMax hinge_motor;
    private final RelativeEncoder hinge_encoder;
    private final AbsoluteEncoder hinge_absolute_encoder;
    // private final DutyCycleEncoder hinge_absolute_encoder;
    private final SparkClosedLoopController hinge_controller;
    private final Debouncer hinge_connected_debounce = new Debouncer(0.5);

    private boolean lift_brake_mode = true;

    public ElevatorIOReal() {
        this.lift_motor = new SparkMax(LIFT_MOTOR_CAN_ID, MotorType.kBrushless);
        this.lift_encoder = this.lift_motor.getEncoder();
        this.lift_controller = this.lift_motor.getClosedLoopController();

        // this.hinge_absolute_encoder = new DutyCycleEncoder(HINGE_ABS_ENCODER_MOTOR_DIO_ID, Math.PI * 2, HINGE_ABSOLUTE_ENCODER_ZERO_POSITION);
        this.hinge_motor = new SparkMax(HINGE_MOTOR_CAN_ID, MotorType.kBrushless);
        this.hinge_encoder = this.hinge_motor.getEncoder();
        this.hinge_absolute_encoder = this.hinge_motor.getAbsoluteEncoder();
        this.hinge_controller = this.hinge_motor.getClosedLoopController();

        SparkUtil.configureSparkMax(this.lift_motor, DEFAULT_LIFT_SPARK_CONFIG);
        SparkUtil.setPosition(this.lift_motor, this.lift_encoder, 0.0);

        // hinge_absolute_encoder.setInverted(true);

        final SparkMaxConfig hinge_config = new SparkMaxConfig();
        SparkUtil.setSparkBaseConfig(hinge_config, HINGE_MOTOR_CURRENT_LIMIT);
        hinge_config.inverted(true);
        hinge_config.absoluteEncoder.inverted(true);
        hinge_config.absoluteEncoder.positionConversionFactor(Math.PI * 2);
        hinge_config.absoluteEncoder.velocityConversionFactor((Math.PI * 2) / 60);
        hinge_config.absoluteEncoder.zeroOffset(0.01753339357674122);
        hinge_config.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0.0, Math.PI * 2.0)
            .pidf(HINGE_P, 0.005, HINGE_D, 0.0)
            .iMaxAccum(0.008)
            .iZone(0.22);
        SparkUtil.configureSparkMax(this.hinge_motor, hinge_config);
        SparkUtil.setPosition(this.hinge_motor, this.hinge_encoder, 0.0);
    }

    @Override public void updateInputs(final ElevatorIOInputs inputs) {
        SparkUtil.spark_sticky_fault = false;

        SparkUtil.ifOk(lift_motor, lift_encoder::getPosition, (value) -> inputs.lift_position_inches = value);
        SparkUtil.ifOk(lift_motor, lift_encoder::getVelocity, (value) -> inputs.lift_velocity_inches_per_second = value);
        SparkUtil.ifOk(lift_motor, new DoubleSupplier[] { lift_motor::getAppliedOutput, lift_motor::getBusVoltage }, (values) -> inputs.lift_applied_volts = values[0] * values[1]);
        SparkUtil.ifOk(lift_motor, lift_motor::getOutputCurrent, (value) -> inputs.lift_current_amps = value);
        inputs.lift_connected = lift_connected_debounce.calculate(!SparkUtil.spark_sticky_fault);

        SparkUtil.spark_sticky_fault = false;
        SparkUtil.ifOk(hinge_motor, hinge_absolute_encoder::getPosition, (value) -> inputs.hinge_absolute_position_radians = value);
        SparkUtil.ifOk(hinge_motor, hinge_encoder::getPosition, (value) -> inputs.hinge_position_radians = value);
        SparkUtil.ifOk(hinge_motor, hinge_encoder::getVelocity, (value) -> inputs.hinge_velocity_radians_per_second = value);
        SparkUtil.ifOk(hinge_motor, new DoubleSupplier[] { hinge_motor::getAppliedOutput, hinge_motor::getBusVoltage }, (values) -> inputs.hinge_applied_volts = values[0] * values[1]);
        SparkUtil.ifOk(hinge_motor, hinge_motor::getOutputCurrent, (value) -> inputs.hinge_current_amps = value);
        inputs.hinge_connected = hinge_connected_debounce.calculate(!SparkUtil.spark_sticky_fault);
        // inputs.hinge_absolute_encoder_connected = hinge_connected_debounce.calculate(hinge_absolute_encoder.isConnected());
    }

    @Override public void setLiftOpenLoop(final double output){
        lift_motor.setVoltage(output);
    }

    @Override public void setLiftPosition(double lift_setpoint_position_inches) {
        if (lift_setpoint_position_inches < LIFT_CLAMP_MIN_POSITION)
            lift_setpoint_position_inches = LIFT_CLAMP_MIN_POSITION;
        if (lift_setpoint_position_inches > Elevator.ELEVATOR_MAX_HEIGHT_INCHES)
            lift_setpoint_position_inches = Elevator.ELEVATOR_MAX_HEIGHT_INCHES;
        lift_controller.setReference(lift_setpoint_position_inches, ControlType.kPosition);
    }

    @Override public void setLiftBrakeMode(final boolean brake_mode) {
        if (lift_brake_mode == brake_mode)
            return;
        lift_brake_mode = brake_mode;
        DEFAULT_LIFT_SPARK_CONFIG.idleMode(brake_mode ? IdleMode.kBrake : IdleMode.kCoast);
        SparkUtil.configureSparkMaxAsyncNonPersist(lift_motor, DEFAULT_LIFT_SPARK_CONFIG);
    }

    @Override public void zeroLiftPosition() {
        SparkUtil.setPosition(lift_motor, lift_encoder, 0.0);
    }

    @Override public void setHingeAngle(final double radians) {
        final double updated_radians = radians <= 0.02 ? 0.02 : radians;
        Logger.recordOutput("Elevator/HingeSetpoint", updated_radians);
        hinge_controller.setReference(updated_radians, ControlType.kPosition);
    }

    @Override public void setHingeOpenLoop(final double output) {
        hinge_motor.setVoltage(output);
    }

    private static SparkMaxConfig defaultLiftSparkConfig() {
        final SparkMaxConfig lift_config = new SparkMaxConfig();
        lift_config.softLimit
            .forwardSoftLimit(Elevator.ELEVATOR_MAX_HEIGHT_INCHES)
            .forwardSoftLimitEnabled(true)
            .reverseSoftLimit(0.25)
            .reverseSoftLimitEnabled(true);
        SparkUtil.setSparkBaseConfig(lift_config, LIFT_MOTOR_CURRENT_LIMIT);
        SparkUtil.setSparkEncoderConfig(lift_config.encoder, LIFT_ENCODER_POSITION_FACTOR, LIFT_ENCODER_VELOCITY_FACTOR);
        SparkUtil.setSparkSignalsConfig(lift_config.signals, 20);
        lift_config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(LIFT_EMPTY_P, 0.0, LIFT_EMPTY_D, 0.0);
        return lift_config;
    }
}
