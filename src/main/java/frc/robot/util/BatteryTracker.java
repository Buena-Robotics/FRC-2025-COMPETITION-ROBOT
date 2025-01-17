package frc.robot.util;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Config;
import frc.robot.Config.RobotMode;

public class BatteryTracker {
    private static final double LOW_BATTERY_VOLTAGE = 10.0;
    private static final double LOW_BATTERY_DISABLED_TIME = 1.5;

    private static final List<Config.RobotType> SUPPORTED_ROBOTS = List.of(Config.RobotType.ROBOT_2025_COMPETION);
    private static final String BATTERY_NAME_FILE = "/home/lvuser/battery-name.txt";
    private static final String DEFAULT_NAME = "0000-000";

    private static final int name_length = 8;
    private static final byte[] scan_command = new byte[] { 0x7e, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, (byte) 0xab, (byte) 0xcd };
    private static final byte[] response_prefix = new byte[] { 0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31 };
    private static final int full_response_length = response_prefix.length + name_length;

    private static String name = DEFAULT_NAME;
    private static boolean battery_name_written = false;

    private static boolean shouldScanBattery() {
        return Config.ROBOT_MODE == RobotMode.REAL && SUPPORTED_ROBOTS.contains(Config.ROBOT_TYPE);
    }

    public static boolean isLowVoltage(final Timer disabled_timer) {
        return RobotController.getBatteryVoltage() < LOW_BATTERY_VOLTAGE && disabled_timer.hasElapsed(LOW_BATTERY_DISABLED_TIME);
    }

    public static void tryWriteBatteryName() {
        if (!(Config.ROBOT_MODE == RobotMode.REAL && !battery_name_written && !BatteryTracker.getName().equals(BatteryTracker.DEFAULT_NAME) && DriverStation.isFMSAttached()))
            return;

        battery_name_written = true;
        try {
            FileWriter fileWriter = new FileWriter(BATTERY_NAME_FILE);
            fileWriter.write(BatteryTracker.getName());
            fileWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Scans the battery. This should be called before the first loop cycle
     *
     * @param timeout
     *            The time to wait before giving up
     */
    public static String scanBattery(final double timeout) {
        if (!shouldScanBattery())
            return name;
        // Only scan on supported robots and in real mode
        try (final SerialPort port = new SerialPort(9600, SerialPort.Port.kUSB)) {
            port.setTimeout(timeout);
            port.setWriteBufferSize(scan_command.length);
            port.setReadBufferSize(full_response_length);

            port.write(scan_command, scan_command.length);
            final byte[] response = port.read(full_response_length);

            // Ensure response is correct length
            if (response.length != full_response_length) {
                System.out.println(
                    "[BatteryTracker] Expected " + full_response_length + " bytes from scanner, got " + response.length);
                return name;
            }

            // Ensure response starts with prefix
            for (int i = 0; i < response_prefix.length; i++) {
                if (response[i] == response_prefix[i])
                    continue; // Doing Good
                System.out.println("[BatteryTracker] Invalid prefix from scanner.  Got data:");
                System.out.println("[BatteryTracker] " + Arrays.toString(response));
                return name;
            }

            // Read name from data
            final byte[] battery_name_bytes = new byte[name_length];
            System.arraycopy(response, response_prefix.length, battery_name_bytes, 0, name_length);
            name = new String(battery_name_bytes);
            System.out.println("[BatteryTracker] Scanned battery " + name);

        } catch (Exception e) {
            System.out.println("[BatteryTracker] Exception while trying to scan battery");
            e.printStackTrace();
        }
        return name;
    }

    /** Returns the name of the last scanned battery. */
    private static String getName() {
        return name;
    }

    /**
     * Returns true if the battery is the same as the last; Returns false if
     * not in competition or
     * if not on a real robot or
     * if there is not a previous battery_name
     */
    public static boolean sameBatteryCheck() {
        if (!shouldScanBattery() ||
            BatteryTracker.getName().equals(BatteryTracker.DEFAULT_NAME))
            return false;

        // Check for battery alert
        final File file = new File(BATTERY_NAME_FILE);
        if (!file.exists())
            return false;

        // Read previous battery name
        String previous_battery_name = "";
        try {
            previous_battery_name = new String(Files.readAllBytes(Paths.get(BATTERY_NAME_FILE)), StandardCharsets.UTF_8);
        } catch (IOException e) {
            System.out.printf("[BatteryTracker] Unable to read previous_battery_name from file");
            e.printStackTrace();
        }

        if (previous_battery_name.equals(BatteryTracker.getName()))
            return true;
        else
            file.delete(); // New battery, delete file
        return false;
    }
}
