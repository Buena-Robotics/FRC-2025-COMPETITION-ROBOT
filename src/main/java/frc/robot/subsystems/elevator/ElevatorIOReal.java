package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.util.SparkUtil;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorIOReal implements ElevatorIO {
    private static final int LIFT_MOTOR_CAN_ID = 9;
    private static final double LIFT_P = 0.2;
    private static final double LIFT_D = 0.0;
    private static final int LIFT_MOTOR_CURRENT_LIMIT = 50;
    private static final double LIFT_ENCODER_POSITION_FACTOR = 1.0;
    private static final double LIFT_ENCODER_VELOCITY_FACTOR = 1.0;

    private final SparkMax lift_motor;
    private final RelativeEncoder lift_encoder;
    private final SparkClosedLoopController lift_controller;
    private final Debouncer lift_connected_debounce = new Debouncer(0.5);

    public ElevatorIOReal() {
        this.lift_motor = new SparkMax(LIFT_MOTOR_CAN_ID, MotorType.kBrushless);
        this.lift_encoder = this.lift_motor.getEncoder();
        this.lift_controller = this.lift_motor.getClosedLoopController();

        final SparkMaxConfig lift_config = new SparkMaxConfig();
        SparkUtil.setSparkBaseConfig(lift_config, LIFT_MOTOR_CURRENT_LIMIT);
        SparkUtil.setSparkEncoderConfig(lift_config.encoder, LIFT_ENCODER_POSITION_FACTOR, LIFT_ENCODER_VELOCITY_FACTOR);
        SparkUtil.setSparkSignalsConfig(lift_config.signals, 20);
        lift_config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(LIFT_P, 0.0, LIFT_D, 0.0);
        SparkUtil.configureSparkMax(lift_motor, lift_config);
        SparkUtil.setPosition(lift_motor, lift_encoder, 0.0);
    }

    @Override public void updateInputs(final ElevatorIOInputs inputs) {
        SparkUtil.spark_sticky_fault = false;
        SparkUtil.ifOk(lift_motor, lift_encoder::getPosition, (value) -> inputs.lift_position_inches = value);
        SparkUtil.ifOk(lift_motor, lift_encoder::getVelocity, (value) -> inputs.lift_velocity_inches_per_second = value);
        SparkUtil.ifOk(lift_motor, new DoubleSupplier[] { lift_motor::getAppliedOutput, lift_motor::getBusVoltage }, (values) -> inputs.lift_applied_volts = values[0] * values[1]);
        SparkUtil.ifOk(lift_motor, lift_motor::getOutputCurrent, (value) -> inputs.lift_current_amps = value);
        inputs.lift_connected = lift_connected_debounce.calculate(!SparkUtil.spark_sticky_fault);
    }

    @Override public void setLiftPosition(final double lift_setpoint_position_inches) {
        lift_controller.setReference(lift_setpoint_position_inches, ControlType.kPosition);
    }
}
