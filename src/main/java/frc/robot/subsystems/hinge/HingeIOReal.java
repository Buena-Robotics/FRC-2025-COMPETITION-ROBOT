package frc.robot.subsystems.hinge;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.filter.Debouncer;
import frc.robot.util.SparkUtil;

public class HingeIOReal implements HingeIO {
    private static final int HINGE_MOTOR_CAN_ID = 14;

    private static final double HINGE_P = 0.5;
    private static final double HINGE_I = 0.005;
    private static final double HINGE_D = 0.2;
    private static final double HINGE_IZONE = 0.22;

    private static final int HINGE_MOTOR_CURRENT_LIMIT = 18;

    private final SparkMax hinge_motor;
    private final RelativeEncoder hinge_encoder;
    private final AbsoluteEncoder hinge_absolute_encoder;
    // private final DutyCycleEncoder hinge_absolute_encoder;
    private final SparkClosedLoopController hinge_controller;
    private final Debouncer hinge_connected_debounce = new Debouncer(0.5);

    public HingeIOReal() {
        // this.hinge_absolute_encoder = new DutyCycleEncoder(HINGE_ABS_ENCODER_MOTOR_DIO_ID, Math.PI * 2, HINGE_ABSOLUTE_ENCODER_ZERO_POSITION);
        this.hinge_motor = new SparkMax(HINGE_MOTOR_CAN_ID, MotorType.kBrushless);
        this.hinge_encoder = this.hinge_motor.getEncoder();
        this.hinge_absolute_encoder = this.hinge_motor.getAbsoluteEncoder();
        this.hinge_controller = this.hinge_motor.getClosedLoopController();

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
            .pidf(HINGE_P, HINGE_I, HINGE_D, 0.0)
            .iMaxAccum(0.008)
            .iZone(HINGE_IZONE);
        SparkUtil.configureSparkMax(this.hinge_motor, hinge_config);
        SparkUtil.setPosition(this.hinge_motor, this.hinge_encoder, 0.0);
    }

    @Override public void updateInputs(final HingeIOInputs inputs) {
        SparkUtil.spark_sticky_fault = false;
        SparkUtil.ifOk(hinge_motor, hinge_absolute_encoder::getPosition, (value) -> inputs.hinge_absolute_position_radians = value);
        SparkUtil.ifOk(hinge_motor, hinge_encoder::getPosition, (value) -> inputs.hinge_position_radians = value);
        SparkUtil.ifOk(hinge_motor, hinge_encoder::getVelocity, (value) -> inputs.hinge_velocity_radians_per_second = value);
        SparkUtil.ifOk(hinge_motor, new DoubleSupplier[] { hinge_motor::getAppliedOutput, hinge_motor::getBusVoltage }, (values) -> inputs.hinge_applied_volts = values[0] * values[1]);
        SparkUtil.ifOk(hinge_motor, hinge_motor::getOutputCurrent, (value) -> inputs.hinge_current_amps = value);
        inputs.hinge_connected = hinge_connected_debounce.calculate(!SparkUtil.spark_sticky_fault);
        // inputs.hinge_absolute_encoder_connected = hinge_connected_debounce.calculate(hinge_absolute_encoder.isConnected());
    }

    @Override public void setHingeAngle(final double radians) {
        final double updated_radians = radians <= 0.02 ? 0.02 : radians;
        Logger.recordOutput("Elevator/HingeSetpoint", updated_radians);
        hinge_controller.setReference(updated_radians, ControlType.kPosition);
    }

    @Override public void setHingeOpenLoop(final double output) {
        hinge_motor.setVoltage(output);
    }
}
