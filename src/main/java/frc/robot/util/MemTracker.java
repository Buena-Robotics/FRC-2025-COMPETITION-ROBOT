package frc.robot.util;

public class MemTracker {
    private static final double MEMORY_USAGE_PERCENT_THRESHOLD = 0.9;

    private static long totalMem() {
        return Runtime.getRuntime().totalMemory();
    }

    private static long freeMem() {
        return Runtime.getRuntime().totalMemory();
    }

    public static boolean highMemoryUsage() {
        return totalMem() / freeMem() >= MEMORY_USAGE_PERCENT_THRESHOLD;
    }
}
