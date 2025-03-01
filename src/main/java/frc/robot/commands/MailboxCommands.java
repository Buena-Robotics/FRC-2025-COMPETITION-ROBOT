package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.mailbox.Mailbox;

public class MailboxCommands {
    private static final double FEED_CORAL_EPSILON = 0.5;
    private static final double TIME_TO_LAUNCH_SECONDS = 0.4;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private MailboxCommands() {}

    public static Command triggerMailboxSpeed(final Mailbox mailbox, final DoubleSupplier speed_supplier) {
        return Commands.run(() -> {
            mailbox.runSpeedSetpoint(speed_supplier.getAsDouble());
        }, mailbox);
    }

    public static Command feedCoral(final Mailbox mailbox) {
        return Commands.runOnce(() -> mailbox.resetPosition(), mailbox).andThen(
            Commands.deadline(new WaitCommand(2.2), Commands.run(() -> mailbox.runPositionSetpoint(Mailbox.FEED_CORAL_POSITION), mailbox)
                .until(() -> {
                    return Math.abs(Mailbox.FEED_CORAL_POSITION - mailbox.getPosition()) < FEED_CORAL_EPSILON;
                })));
    }

    public static Command lockDriveAndLaunchCoral(final Mailbox mailbox, final Drive drive) {
        return Commands.deadline(new WaitCommand(TIME_TO_LAUNCH_SECONDS), Commands.run(() -> {
            mailbox.runSpeedSetpoint(-1.0);
            drive.stop();
        }, mailbox, drive));
    }

    public static Command feedforwardCharacterization(final Mailbox mailbox) {
        List<Double> velocity_samples = new LinkedList<>();
        List<Double> voltage_samples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
            // Reset data
            Commands.runOnce(() -> {
                velocity_samples.clear();
                voltage_samples.clear();
            }),

            // Allow modules to orient
            Commands.run(
                () -> {
                    mailbox.runCharacterization(0.0);
                },
                mailbox)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                () -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    mailbox.runCharacterization(voltage);
                    velocity_samples.add(mailbox.getFFCharacterizationVelocity());
                    voltage_samples.add(voltage);
                },
                mailbox)

                // When cancelled, calculate and print results
                .finallyDo(() -> {
                    int n = velocity_samples.size();
                    double sum_x = 0.0;
                    double sum_y = 0.0;
                    double sum_xy = 0.0;
                    double sum_x2 = 0.0;
                    for (int i = 0; i < n; i++) {
                        sum_x += velocity_samples.get(i);
                        sum_y += voltage_samples.get(i);
                        sum_xy += velocity_samples.get(i) * voltage_samples.get(i);
                        sum_x2 += velocity_samples.get(i) * velocity_samples.get(i);
                    }
                    double ks = (sum_y * sum_x2 - sum_x * sum_xy) / (n * sum_x2 - sum_x * sum_x);
                    double kv = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);

                    NumberFormat formatter = new DecimalFormat("#0.00000");
                    System.out.println("********** Elevator FF Characterization Results **********");
                    System.out.println("\tkS: " + formatter.format(ks));
                    System.out.println("\tkV: " + formatter.format(kv));
                }));
    }
}
