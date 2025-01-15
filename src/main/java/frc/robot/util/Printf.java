package frc.robot.util;

public class Printf {
    public static void info(String msg, Object... args) {
        System.out.printf(msg, args);
    }

    public static void warn(String msg, Object... args) {
        System.out.printf(msg, args);
    }

    public static void error(String msg, Object... args) {
        System.out.printf(msg, args);
    }
}
