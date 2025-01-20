package frc.robot.subsystems.climb;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import frc.robot.util.SparkUtil;

public class ClimbIOReal implements ClimbIO {
    private static final int WINCH_MOTOR_CAN_ID = 10;
    private final SparkMax winch_motor;
    private final RelativeEncoder winch_encoder;
    private final Debouncer winch_connected_debounce = new Debouncer(0.5);

    public ClimbIOReal() {
        this.winch_motor = new SparkMax(WINCH_MOTOR_CAN_ID, MotorType.kBrushless);
        this.winch_encoder = this.winch_motor.getEncoder();
        final SparkMaxConfig winch_config = new SparkMaxConfig();

        SparkUtil.configureSparkMax(this.winch_motor, winch_config);
        SparkUtil.setPosition(this.winch_motor, this.winch_encoder, 0.0);
    }

    @Override public void updateInputs(final ClimbIOInputs inputs) {
        SparkUtil.spark_sticky_fault = false;
        SparkUtil.ifOk(winch_motor, winch_encoder::getPosition, (value) -> inputs.winch_position_radians = value);
        SparkUtil.ifOk(winch_motor, winch_encoder::getVelocity, (value) -> inputs.winch_velocity_radians_per_second = value);
        SparkUtil.ifOk(winch_motor, new DoubleSupplier[] { winch_motor::getAppliedOutput, winch_motor::getBusVoltage }, (values) -> inputs.winch_applied_volts = values[0] * values[1]);
        SparkUtil.ifOk(winch_motor, winch_motor::getOutputCurrent, (value) -> inputs.winch_current_amps = value);
        inputs.winch_connected = winch_connected_debounce.calculate(!SparkUtil.spark_sticky_fault);
    }

    @Override public void setWinchSpeed(final double winch_speed) {
        winch_motor.set(winch_speed);
    }
}
