package frc.robot.subsystems.drive;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.function.DoubleSupplier;

/**
 * Provides an interface for asynchronously reading high-frequency measurements
 * to a set of queues.
 *
 * <p>
 * This version includes an overload for Spark signals, which checks for errors
 * to ensure that all measurements in
 * the sample are valid.
 */
public class SparkOdometryThread {
    private final List<SparkBase> sparks = new ArrayList<>();
    private final List<DoubleSupplier> spark_signals = new ArrayList<>();
    private final List<DoubleSupplier> generic_signals = new ArrayList<>();
    private final List<Queue<Double>> spark_queues = new ArrayList<>();
    private final List<Queue<Double>> generic_queues = new ArrayList<>();
    private final List<Queue<Double>> timestamp_queues = new ArrayList<>();

    private static SparkOdometryThread instance = null;
    private Notifier notifier = new Notifier(this::run);

    public static SparkOdometryThread getInstance() {
        if (instance == null) {
            instance = new SparkOdometryThread();
        }
        return instance;
    }

    private SparkOdometryThread() {
        notifier.setName("OdometryThread");
    }

    public void start() {
        if (timestamp_queues.size() > 0) {
            notifier.startPeriodic(1.0 / Drive.ODOMETRY_FREQUENCY_HERTZ);
        }
    }

    /** Registers a Spark signal to be read from the thread. */
    public Queue<Double> registerSignal(final SparkBase spark, final DoubleSupplier signal) {
        final Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometry_lock.lock();
        try {
            sparks.add(spark);
            spark_signals.add(signal);
            spark_queues.add(queue);
        } finally {
            Drive.odometry_lock.unlock();
        }
        return queue;
    }

    /** Registers a generic signal to be read from the thread. */
    public Queue<Double> registerSignal(final DoubleSupplier signal) {
        final Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometry_lock.lock();
        try {
            generic_signals.add(signal);
            generic_queues.add(queue);
        } finally {
            Drive.odometry_lock.unlock();
        }
        return queue;
    }

    /** Returns a new queue that returns timestamp values for each sample. */
    public Queue<Double> makeTimestampQueue() {
        final Queue<Double> queue = new ArrayBlockingQueue<>(20);
        Drive.odometry_lock.lock();
        try {
            timestamp_queues.add(queue);
        } finally {
            Drive.odometry_lock.unlock();
        }
        return queue;
    }

    private void run() {
        // Save new data to queues
        Drive.odometry_lock.lock();
        try {
            // Get sample timestamp
            final double timestamp = RobotController.getFPGATime() / 1e6;

            // Read Spark values, mark invalid in case of error
            final double[] spark_values = new double[spark_signals.size()];
            boolean is_valid = true;
            for (int i = 0; i < spark_signals.size(); i++) {
                spark_values[i] = spark_signals.get(i).getAsDouble();
                if (sparks.get(i).getLastError() != REVLibError.kOk) {
                    is_valid = false;
                }
            }

            // If valid, add values to queues
            if (is_valid) {
                for (int i = 0; i < spark_signals.size(); i++) {
                    spark_queues.get(i).offer(spark_values[i]);
                }
                for (int i = 0; i < generic_signals.size(); i++) {
                    generic_queues.get(i).offer(generic_signals.get(i).getAsDouble());
                }
                for (int i = 0; i < timestamp_queues.size(); i++) {
                    timestamp_queues.get(i).offer(timestamp);
                }
            }
        } finally {
            Drive.odometry_lock.unlock();
        }
    }
}
