package frc.robot.subsystems.mailbox;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.SparkUtil;

public class MailboxIOReal implements MailboxIO {
    private static final int SHOOTER_MOTOR_CAN_ID = 11;
    private static final double SHOOTER_S = 0.21830;
    private static final double SHOOTER_V = 0.00105;

    private static final int CORAL_BEAMBREAK_DIO_ID = 0;

    private final SparkMax shooter_motor;
    private final RelativeEncoder shooter_encoder;
    private final SparkClosedLoopController shooter_controller;
    private final Debouncer shooter_connected_debounce = new Debouncer(0.5);

    private final DigitalInput coral_beambreak = new DigitalInput(CORAL_BEAMBREAK_DIO_ID);
    private final Debouncer coral_beambreak_connected_debounce = new Debouncer(0.5);


    public MailboxIOReal() {
        this.shooter_motor = new SparkMax(SHOOTER_MOTOR_CAN_ID, MotorType.kBrushless);
        this.shooter_encoder = this.shooter_motor.getEncoder();
        this.shooter_controller = this.shooter_motor.getClosedLoopController();

        final SparkMaxConfig shooter_config = new SparkMaxConfig();
        shooter_config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pidf(0.03, 0.00, 0.8, 0.0);

        SparkUtil.configureSparkMax(this.shooter_motor, shooter_config);
        SparkUtil.setPosition(this.shooter_motor, this.shooter_encoder, 0.0);
    }

    public void updateInputs(final MailboxIOInputs inputs) {
        SparkUtil.spark_sticky_fault = false;
        SparkUtil.ifOk(shooter_motor, shooter_encoder::getPosition, (value) -> inputs.shooter_position_radians = value);
        SparkUtil.ifOk(shooter_motor, shooter_encoder::getVelocity, (value) -> inputs.shooter_velocity_radians_per_second = value);
        SparkUtil.ifOk(shooter_motor, new DoubleSupplier[] { shooter_motor::getAppliedOutput, shooter_motor::getBusVoltage }, (values) -> inputs.shooter_applied_volts = values[0] * values[1]);
        SparkUtil.ifOk(shooter_motor, shooter_motor::getOutputCurrent, (value) -> inputs.shooter_current_amps = value);
        inputs.shooter_connected = shooter_connected_debounce.calculate(!SparkUtil.spark_sticky_fault);

        inputs.coral_beam_broken = !coral_beambreak.get();
        inputs.coral_beambreak_connected = coral_beambreak_connected_debounce.calculate(true);
    }

    @Override public void resetPosition() {
        SparkUtil.setPosition(shooter_motor, shooter_encoder, 0.0);
    }

    @Override public void setShooterOpenLoop(final double output){
        shooter_motor.setVoltage(output);
    }

    @Override public void setShooterSpeed(final double shooter_speed) {
        shooter_motor.set(shooter_speed);
    }

    @Override public void setShooterPosition(final double shooter_position_radians) {
        final double ffVolts = SHOOTER_S * Math.signum(shooter_position_radians) + SHOOTER_V * shooter_position_radians;
        shooter_controller.setReference(shooter_position_radians, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }

    @Override public void setShooterVelocity(final double shooter_velocity_radians_per_second) {
        final double ffVolts = SHOOTER_S * Math.signum(shooter_velocity_radians_per_second) + SHOOTER_V * shooter_velocity_radians_per_second;
        shooter_controller.setReference(shooter_velocity_radians_per_second, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ffVolts, ArbFFUnits.kVoltage);
    }
}
