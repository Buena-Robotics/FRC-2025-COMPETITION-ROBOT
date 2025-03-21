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
import frc.robot.subsystems.hinge.Hinge;
import frc.robot.subsystems.hinge.Hinge.HingeSetpoint;

public class HingeCommands {
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private HingeCommands() {}

    public static Command triggerHingeAngle(final Hinge hinge, final DoubleSupplier hinge_angle_supplier) {
        return Commands.run(
            () -> {
                hinge.runHingeSetpoint(hinge_angle_supplier.getAsDouble() * 2.3);
            }, hinge);
    }

    // public static Command grabAlgae(final Elevator elevator, final Supplier<ElevatorSetpoint> algae_height) {
    //     return Commands.run(() -> {
    //         elevator.runLiftSetpoint(ElevatorSetpoint.BOTTOM.getValue());
    //         elevator.runHingeSetpoint(HingeSetpoint.ALGAE.getValue());
    //     }, elevator)
    //         .until(() -> elevator.isLiftAtSetpoint(ElevatorSetpoint.BOTTOM) && elevator.isHingeAtSetpoint(HingeSetpoint.ALGAE))
    //         .andThen(new WaitCommand(1.0))
    //         .andThen(Commands.run(() -> {
    //             elevator.runLiftSetpoint(algae_height.get().getValue());
    //         }, elevator)
    //             .until(() -> elevator.isLiftAtSetpoint(algae_height.get()))).andThen(new WaitCommand(5.0))
    //         .andThen(Commands.run(() -> {
    //             elevator.runLiftSetpoint(ElevatorSetpoint.BOTTOM.getValue());
    //         }, elevator)
    //             .until(() -> elevator.isLiftAtSetpoint(ElevatorSetpoint.BOTTOM)));
    // }

    public static Command releaseAlgae(final Hinge hinge) {
        return Commands.deadline(new WaitCommand(3.0), Commands.run(() -> {
            hinge.runHingeSetpoint(HingeSetpoint.RELEASE_ALGAE.getValue());
        }, hinge)).andThen(Commands.runOnce(() -> {
            hinge.runHingeSetpoint(HingeSetpoint.TOP.getValue());
        }, hinge));
    }

    public static Command feedforwardCharacterization(final Hinge hinge) {
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
                    hinge.runHingeCharacterization(0.0);
                },
                hinge)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                () -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    hinge.runHingeCharacterization(voltage);
                    velocity_samples.add(hinge.getHingeFFCharacterizationVelocity());
                    voltage_samples.add(voltage);
                },
                hinge)

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
