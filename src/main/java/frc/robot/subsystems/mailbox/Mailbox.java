package frc.robot.subsystems.mailbox;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.FieldConstants;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorSetpoint;

public class Mailbox extends SubsystemBase {
    public static final double FEED_CORAL_POSITION = -25.0;

    private final MailboxIO io;
    private final Alert shooter_disconnect_alert = new Alert("Disconnected shooter motor", AlertType.kError);
    private final Alert coral_beambreak_disconnect_alert = new Alert("Disconnected coral beambreak sensor", AlertType.kWarning);
    private final SysIdRoutine sys_id;

    private final MailboxIOInputsAutoLogged inputs = new MailboxIOInputsAutoLogged();

    private final Elevator elevator;

    public Mailbox(final MailboxIO io, final Elevator elevator) {
        this.elevator = elevator;
        this.io = io;
        this.sys_id = new SysIdRoutine(
            new SysIdRoutine.Config(Volts.of(0.01).per(Second), Volts.of(0.01), Seconds.of(60), (state) -> Logger.recordOutput("Elevator/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Mailbox", inputs);

        Logger.recordOutput("Mailbox/ClosetReefBranchStats", getClosestReefBranchStats());
        Logger.recordOutput("Mailbox/GoodShot", goodShot());

        shooter_disconnect_alert.set(!inputs.shooter_connected);
        coral_beambreak_disconnect_alert.set(!inputs.coral_beambreak_connected);
    }

    public Command sysIdQuasistatic(final SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sys_id.quasistatic(direction));
    }

    public Command sysIdDynamic(final SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sys_id.dynamic(direction));
    }

    public void runCharacterization(final double output) {
        io.setShooterOpenLoop(output);
    }

    public boolean coralWaiting() {
        return inputs.coral_beam_broken;
    }

    private BranchCloseStats getClosestReefBranchStats() {
        int closest_index = 0;
        double closest_distance = Double.MAX_VALUE;
        Pose3d[] pose_list = FieldConstants.REEF_BRANCHES_POSES();
        for (int i = 0; i < pose_list.length; i++) {
            double distance = elevator.virtualCameraPosition().getTranslation().getDistance(pose_list[i].getTranslation());
            if (distance < closest_distance) {
                closest_distance = distance;
                closest_index = i;
            }
        }
        return new BranchCloseStats(pose_list[closest_index], Units.metersToInches(closest_distance), pose_list[closest_index].getRotation().minus(new Rotation3d(0, 0, Math.PI)).rotateBy(elevator.virtualCameraPosition().getRotation()));
    }

    private boolean goodShot() {
        final BranchCloseStats stats = getClosestReefBranchStats();
        return Math.abs(stats.distance_inches) < 5.0 && Units.radiansToDegrees(Math.abs(stats.rotation.getZ())) < 2.0 && (elevator.isLiftAtSetpoint(ElevatorSetpoint.L2) || elevator.isLiftAtSetpoint(ElevatorSetpoint.L3));
    }

    public double getPosition() {
        return inputs.shooter_position_radians;
    }

    public double getFFCharacterizationVelocity() {
        return inputs.shooter_velocity_radians_per_second;
    }

    public void resetPosition() {
        io.resetPosition();
    }

    public void runSpeedSetpoint(final double shooter_speed) {
        Logger.recordOutput("Mailbox/Speedsetpoint", shooter_speed);
        io.setShooterSpeed(shooter_speed);
    }

    public void runPositionSetpoint(final double shooter_position_radians) {
        Logger.recordOutput("Mailbox/PositionSetpoint", shooter_position_radians);
        io.setShooterPosition(shooter_position_radians);
    }

    public void runVelocitySetpoint(final double shooter_velocity_radians_per_second) {
        Logger.recordOutput("Mailbox/VelocitySetpoint", shooter_velocity_radians_per_second);
        io.setShooterVelocity(shooter_velocity_radians_per_second);
    }

    public static record BranchCloseStats(Pose3d pose, double distance_inches, Rotation3d rotation) {}
}
