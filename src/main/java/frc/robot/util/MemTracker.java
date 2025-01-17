package frc.robot.util;

public class MemTracker {
    private static final double MEMORY_USAGE_PERCENT_THRESHOLD = 0.7;

    private static long totalMem() {
        return Runtime.getRuntime().totalMemory();
    }

    private static long freeMem() {
        return Runtime.getRuntime().freeMemory();
    }
    public static double memoryUsage(){
        return (double)freeMem() / (double)totalMem();
    }

    public static boolean highMemoryUsage() {
        return memoryUsage() >= MEMORY_USAGE_PERCENT_THRESHOLD;
    }
}
