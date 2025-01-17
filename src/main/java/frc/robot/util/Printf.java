package frc.robot.util;

public class Printf {
    public static void info(String msg, Object... args) {
        System.out.printf("[INFO]: " + msg + "\n", args);
    }

    public static void warn(String msg, Object... args) {
        System.out.printf("[WARN]: " + msg + "\n", args);
    }

    public static void error(String msg, Object... args) {
        System.out.printf("[ERROR]: " + msg + "\n", args);
    }
}
