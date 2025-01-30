// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SignalsConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;

public class SparkUtil {
    /** Stores whether any error was has been detected by other utility methods. */
    public static boolean spark_sticky_fault = false;

    /** Processes a value from a Spark only if the value is valid. */
    public static void ifOk(final SparkBase spark, final DoubleSupplier supplier, final DoubleConsumer consumer) {
        double value = supplier.getAsDouble();
        if (spark.getLastError() == REVLibError.kOk) {
            consumer.accept(value);
        } else {
            spark_sticky_fault = true;
        }
    }

    /** Processes a value from a Spark only if the value is valid. */
    public static void ifOk(final SparkBase spark, final DoubleSupplier[] suppliers, final Consumer<double[]> consumer) {
        double[] values = new double[suppliers.length];
        for (int i = 0; i < suppliers.length; i++) {
            values[i] = suppliers[i].getAsDouble();
            if (spark.getLastError() != REVLibError.kOk) {
                spark_sticky_fault = true;
                return;
            }
        }
        consumer.accept(values);
    }

    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(final SparkBase spark, final Supplier<REVLibError> command) {
        tryUntilOk(spark, 5, command);
    }

    /** Attempts to run the command until no error is produced. */
    public static void tryUntilOk(final SparkBase spark, final int max_attempts, final Supplier<REVLibError> command) {
        for (int i = 0; i < max_attempts; i++) {
            REVLibError error = command.get();
            if (error == REVLibError.kOk) {
                break;
            } else {
                spark_sticky_fault = true;
            }
        }
    }

    public static void configureSparkMax(final SparkBase spark, final SparkMaxConfig config) {
        SparkUtil.tryUntilOk(spark, () -> spark.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    }

    public static void configureSparkMaxAsyncNonPersist(final SparkBase spark, final SparkMaxConfig config) {
        SparkUtil.tryUntilOk(spark, () -> spark.configureAsync(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters));
    }

    public static void setPosition(final SparkBase spark, final RelativeEncoder encoder, final double position) {
        SparkUtil.tryUntilOk(spark, () -> encoder.setPosition(position));
    }

    public static double[] getSimulationOdometryTimeStamps() {
        final double[] odometry_timestamps = new double[SimulatedArena.getSimulationSubTicksIn1Period()];
        for (int i = 0; i < odometry_timestamps.length; i++) {
            odometry_timestamps[i] = Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().in(Seconds);
        }
        return odometry_timestamps;
    }

    public static SparkMaxConfig setSparkBaseConfig(final SparkMaxConfig config, final int current_limit) {
        config.idleMode(IdleMode.kBrake)
            .smartCurrentLimit(current_limit)
            .voltageCompensation(12.0);
        return config;
    }

    public static EncoderConfig setSparkEncoderConfig(final EncoderConfig encoder, final double position_factor, final double velocity_factor) {
        encoder.positionConversionFactor(position_factor)
            .velocityConversionFactor(velocity_factor)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(2);
        return encoder;
    }

    public static SignalsConfig setSparkSignalsConfig(final SignalsConfig signals, final int period_ms) {
        signals.primaryEncoderPositionAlwaysOn(true)
            // .primaryEncoderPositionPeriodMs((int) (1000.0 /
            // Drive.ODOMETRY_FREQUENCY_HERTZ))
            .primaryEncoderPositionPeriodMs(period_ms)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        return signals;
    }
}
