package frc.robot.util;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class CanTracker {
    private static final double CAN_ERROR_TIME_THRESHOLD_SECONDS = 0.5; // Seconds to disable alert

    public static boolean hasCanErrors(final Timer can_error_timer, final Timer can_error_timer_initial) {
        final CANStatus can_status = RobotController.getCANStatus();
        if (can_status.receiveErrorCount > 0 || can_status.transmitErrorCount > 0)
            can_error_timer.reset();

        return !can_error_timer.hasElapsed(CAN_ERROR_TIME_THRESHOLD_SECONDS) && can_error_timer_initial.hasElapsed(CAN_ERROR_TIME_THRESHOLD_SECONDS);
    }
}
