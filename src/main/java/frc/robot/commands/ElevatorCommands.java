package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorSetpoint;

public class ElevatorCommands {
    private static final double SHAKE_DELAY_SECONDS = 0.3;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

    private ElevatorCommands() {}

    private static List<Pair<Double, ElevatorSetpoint>> estimate_setpoint_pairs = List.of(
        new Pair<>(0.00, ElevatorSetpoint.CORAL_STATION),
        new Pair<>(0.20, ElevatorSetpoint.L2),
        new Pair<>(0.40, ElevatorSetpoint.L3));

    // estimate is between 0-1
    private static ElevatorSetpoint closestSetpoint(final double estimate) {
        int closest_index = 0;
        double closest_distance = Double.MAX_VALUE;
        for (int i = 0; i < estimate_setpoint_pairs.size(); i++) {
            double distance = Math.abs(estimate - estimate_setpoint_pairs.get(i).getFirst());
            if (distance < closest_distance) {
                closest_distance = distance;
                closest_index = i;
            }
        }
        return estimate_setpoint_pairs.get(closest_index).getSecond();
    }

    public static Command triggerElevatorHeightAndSetpoint(final Elevator elevator, final BooleanSupplier setpoint_mode_supplier, final DoubleSupplier height_supplier, final DoubleSupplier setpoint_estimate_supplier) {
        return Commands.run(
            () -> {
                if (setpoint_mode_supplier.getAsBoolean()) {
                    final ElevatorSetpoint closest = closestSetpoint(setpoint_estimate_supplier.getAsDouble());
                    elevator.runLiftSetpoint(closest.getValue());
                } else {
                    elevator.runLiftSetpoint(height_supplier.getAsDouble() * Elevator.ELEVATOR_MAX_HEIGHT_INCHES);
                }
            }, elevator);
    }

    public static Command triggerElevatorHeight(final Elevator elevator, final DoubleSupplier height_supplier) {
        return Commands.run(
            () -> {
                elevator.runLiftSetpoint(height_supplier.getAsDouble() * Elevator.ELEVATOR_MAX_HEIGHT_INCHES);
            }, elevator);
    }

    public static Command triggerElevatorSetpoint(final Elevator elevator, final ElevatorSetpoint setpoint) {
        return Commands.runOnce(
            () -> {
                elevator.runLiftSetpoint(setpoint.getValue());
            }, elevator);
    }

    public static Command releaseAlgae(final Elevator elevator) {
        return Commands.run(() -> {}, elevator);
    }

    public static Command shakeElevator(final Elevator elevator) {
        return Commands.sequence(
            new InstantCommand(() -> {
                elevator.runLiftSetpoint(Elevator.ELEVATOR_MAX_HEIGHT_INCHES);
            }, elevator),
            new WaitCommand(SHAKE_DELAY_SECONDS),
            new InstantCommand(() -> {
                elevator.runLiftSetpoint(0.0);
            }));
    }

    public static Command feedforwardCharacterization(final Elevator elevator) {
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
                    elevator.runLiftCharacterization(0.0);
                },
                elevator)
                .withTimeout(FF_START_DELAY),

            // Start timer
            Commands.runOnce(timer::restart),

            // Accelerate and gather data
            Commands.run(
                () -> {
                    double voltage = timer.get() * FF_RAMP_RATE;
                    elevator.runLiftCharacterization(voltage);
                    velocity_samples.add(elevator.getLiftFFCharacterizationVelocity());
                    voltage_samples.add(voltage);
                },
                elevator)

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
