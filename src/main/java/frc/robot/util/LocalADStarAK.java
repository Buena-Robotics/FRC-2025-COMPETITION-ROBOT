package frc.robot.util;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class LocalADStarAK implements Pathfinder {
    private final ADStarIO io = new ADStarIO();

    /**
     * Get if a new path has been calculated since the last time a path was
     * retrieved
     *
     * @return True if a new path is available
     */
    @Override
    public boolean isNewPathAvailable() {
        if (!Logger.hasReplaySource()) {
            io.updateIsNewPathAvailable();
        }

        Logger.processInputs("LocalADStarAK", io);

        return io.is_new_path_available;
    }

    /**
     * Get the most recently calculated path
     *
     * @param constraints
     *            The path constraints to use when creating the path
     * @param goal_end_state
     *            The goal end state to use when creating the path
     * @return The PathPlannerPath created from the points calculated by the
     *         pathfinder
     */
    @Override
    public PathPlannerPath getCurrentPath(PathConstraints constraints, GoalEndState goal_end_state) {
        if (!Logger.hasReplaySource()) {
            io.updateCurrentPathPoints(constraints, goal_end_state);
        }

        Logger.processInputs("LocalADStarAK", io);

        if (io.current_path_points.isEmpty()) {
            return null;
        }

        return PathPlannerPath.fromPathPoints(io.current_path_points, constraints, goal_end_state);
    }

    /**
     * Set the start position to pathfind from
     *
     * @param start_position
     *            Start position on the field. If this is within an obstacle it will
     *            be
     *            moved to the nearest non-obstacle node.
     */
    @Override
    public void setStartPosition(Translation2d start_position) {
        if (!Logger.hasReplaySource()) {
            io.ad_star.setStartPosition(start_position);
        }
    }

    /**
     * Set the goal position to pathfind to
     *
     * @param goal_position
     *            Goal position on the field. f this is within an obstacle it will
     *            be moved
     *            to the nearest non-obstacle node.
     */
    @Override
    public void setGoalPosition(Translation2d goal_position) {
        if (!Logger.hasReplaySource()) {
            io.ad_star.setGoalPosition(goal_position);
        }
    }

    /**
     * Set the dynamic obstacles that should be avoided while pathfinding.
     *
     * @param obs
     *            A List of Translation2d pairs representing obstacles. Each
     *            Translation2d represents
     *            opposite corners of a bounding box.
     * @param current_robot_position
     *            The current position of the robot. This is needed to change the
     *            start
     *            position of the path to properly avoid obstacles
     */
    @Override
    public void setDynamicObstacles(
            List<Pair<Translation2d, Translation2d>> obs, Translation2d current_robot_position) {
        if (!Logger.hasReplaySource()) {
            io.ad_star.setDynamicObstacles(obs, current_robot_position);
        }
    }

    private static class ADStarIO implements LoggableInputs {
        public LocalADStar ad_star = new LocalADStar();
        public boolean is_new_path_available = false;
        public List<PathPoint> current_path_points = Collections.emptyList();

        @Override
        public void toLog(LogTable table) {
            table.put("IsNewPathAvailable", is_new_path_available);

            double[] points_logged = new double[current_path_points.size() * 2];
            int idx = 0;
            for (PathPoint point : current_path_points) {
                points_logged[idx] = point.position.getX();
                points_logged[idx + 1] = point.position.getY();
                idx += 2;
            }

            table.put("CurrentPathPoints", points_logged);
        }

        @Override
        public void fromLog(LogTable table) {
            is_new_path_available = table.get("IsNewPathAvailable", false);

            double[] points_logged = table.get("CurrentPathPoints", new double[0]);

            List<PathPoint> path_points = new ArrayList<>();
            for (int i = 0; i < points_logged.length; i += 2) {
                path_points.add(
                        new PathPoint(new Translation2d(points_logged[i], points_logged[i + 1]), null));
            }

            current_path_points = path_points;
        }

        public void updateIsNewPathAvailable() {
            is_new_path_available = ad_star.isNewPathAvailable();
        }

        public void updateCurrentPathPoints(PathConstraints constraints, GoalEndState goal_end_state) {
            PathPlannerPath current_path = ad_star.getCurrentPath(constraints, goal_end_state);

            if (current_path != null) {
                current_path_points = current_path.getAllPathPoints();
            } else {
                current_path_points = Collections.emptyList();
            }
        }
    }
}