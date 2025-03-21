package frc.robot.subsystems.hinge;

import static edu.wpi.first.units.Units.*;


import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.Utils;

public class Hinge extends SubsystemBase {
    private final HingeIO io;
    private final Alert hinge_disconnect_alert = new Alert("Disconnected hinge motor", AlertType.kError);;
    private final HingeIOInputsAutoLogged inputs = new HingeIOInputsAutoLogged();

    private final SysIdRoutine sys_id;

    public Hinge(final HingeIO io) {
        this.io = io;

        this.sys_id = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.01).per(Second), Volts.of(0.01), Seconds.of(60), (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runHingeCharacterization(voltage.in(Volts)), null, this));
    }

    @Override public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hinge", inputs);

        // Update alerts
        hinge_disconnect_alert.set(!inputs.hinge_connected);
    }

    public void runHingeCharacterization(final double output) {
        io.setHingeOpenLoop(output);
    }

    public double getHingeFFCharacterizationVelocity() {
        return inputs.hinge_velocity_radians_per_second;
    }

    public double getHingePositionRadians() {
        return inputs.hinge_absolute_position_radians;
    }

    public void runHingeSetpoint(final double hinge_position_radians) {
        io.setHingeAngle(hinge_position_radians);
    }

    public boolean isHingeAtSetpoint(final HingeSetpoint setpoint) {
        return Utils.epsilonOf(getHingePositionRadians(), setpoint.getValue(), 0.08);
    }

    public Command hingeSysIdQuasistatic(final SysIdRoutine.Direction direction) {
        return run(() -> runHingeCharacterization(0.0)).withTimeout(1.0).andThen(sys_id.quasistatic(direction));
    }

    public Command hingeSysIdDynamic(final SysIdRoutine.Direction direction) {
        return run(() -> runHingeCharacterization(0.0)).withTimeout(1.0).andThen(sys_id.dynamic(direction));
    }

    public static enum HingeSetpoint {
        TOP(0.0), ALGAE(1.45), RELEASE_ALGAE(2.3);

        private double setpoint_radians = 0.0;

        private HingeSetpoint(final double setpoint_radians) {
            this.setpoint_radians = setpoint_radians;
        }

        public double getValue() {
            return setpoint_radians;
        }
    }
}
