package frc.robot.subsystems.vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SillyServo {
    private final double GUESS_SERVO_P = 6.0;
    private final double GUESS_SERVO_D = 0.2;

    private final Servo servo;

    private final DCMotor guess_gearbox = new DCMotor(5, 0.1765, 1.25, 0.13, Units.rotationsPerMinuteToRadiansPerSecond(1200.0), 1);
    private final DCMotorSim guess_sim = new DCMotorSim(LinearSystemId.createDCMotorSystem(guess_gearbox, 0.004, 1), guess_gearbox);
    private final PIDController guess_controller = new PIDController(GUESS_SERVO_P, 0, GUESS_SERVO_D);

    private double guess_applied_volts = 0.0;

    public SillyServo(final int channel) {
        this.servo = new Servo(channel);
        guess_sim.setAngle(90.0);
    }

    public void periodic() {
        guess_applied_volts = guess_controller.calculate(guess_sim.getAngularPositionRad());

        guess_sim.setInputVoltage(MathUtil.clamp(guess_applied_volts, -5.0, 5.0));
        guess_sim.update(0.02);

        if (guess_sim.getAngularPositionRad() > 180.0)
            guess_sim.setState(175.0, -1.0);
        if (guess_sim.getAngularPositionRad() < 0.0)
            guess_sim.setState(5.0, 1.0);
    }

    public void setAngle(final double angle) {
        guess_controller.setSetpoint(angle);
        servo.setAngle(angle);
    }

    public double getGuessedAngle() {
        return guess_sim.getAngularPositionRad();
    }

    public double getAngle() {
        return servo.getAngle();
    }
}
