package frc.robot.subsystems.mailbox;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.util.SparkUtil;

public class MailboxIOReal implements MailboxIO {
    private static final int SHOOTER_MOTOR_CAN_ID = 10;
    private final SparkMax shooter_motor;
    private final RelativeEncoder shooter_encoder;

    public MailboxIOReal() {
        this.shooter_motor = new SparkMax(SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
        this.shooter_encoder = this.shooter_motor.getEncoder();
        final SparkMaxConfig winch_config = new SparkMaxConfig();

        SparkUtil.configureSparkMax(this.shooter_motor, winch_config);
        SparkUtil.setPosition(this.shooter_motor, this.shooter_encoder, 0.0);
    }

    public void updateInputs(final MailboxIOInputs inputs) {}

    @Override public void setShooterSpeed(final double shooter_speed) {}

    @Override public void setShooterVelocity(final double shooter_velocity) {}
}
