package frc.robot.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import java.util.List;
import org.ironmaple.simulation.SimulatedArena;

public class ArenaSchool2025Reefscape extends SimulatedArena {
    public static final class SchoolReefscapeFieldObstacleMap extends FieldMap {
        public SchoolReefscapeFieldObstacleMap() {
            super();

            final double room_width_meters = Units.inchesToMeters(315.75);
            final double room_length_meters = Units.inchesToMeters(315.75 + 24);

            final Translation2d front_left_corner = new Translation2d(0, 0);
            final Translation2d front_right_corner = new Translation2d(room_width_meters, 0);
            final Translation2d back_left_corner = new Translation2d(0, room_length_meters);
            final Translation2d back_right_corner = new Translation2d(room_width_meters, room_length_meters);

            // Front wall
            super.addBorderLine(front_left_corner, front_right_corner);
            // Back Wall
            super.addBorderLine(back_left_corner, back_right_corner);
            // Left Wall
            super.addBorderLine(front_left_corner, back_left_corner);
            // Right Wall
            super.addBorderLine(front_right_corner, back_right_corner);

            // // blue coral stations
            // super.addBorderLine(new Translation2d(0, 1.270), new Translation2d(1.672,
            // 0));
            // super.addBorderLine(new Translation2d(0, 6.782), new Translation2d(1.672,
            // 8.052));

            // // red wall
            // super.addBorderLine(new Translation2d(17.548, 1.270), new
            // Translation2d(17.548, 6.782));

            // // red coral stations
            // super.addBorderLine(new Translation2d(17.548, 1.270), new
            // Translation2d(17.548 - 1.672, 0));
            // super.addBorderLine(new Translation2d(17.548, 6.782), new
            // Translation2d(17.548 - 1.672, 8.052));

            // // upper walls
            // super.addBorderLine(new Translation2d(1.672, 8.052), new Translation2d(17.548
            // - 1.672, 8.052));

            // // lower walls
            // super.addBorderLine(new Translation2d(1.672, 0), new Translation2d(17.548 -
            // 1.672, 0));

            // // blue reef
            // Translation2d[] reef_vortices = new Translation2d[] {
            // new Translation2d(3.658, 3.546),
            // new Translation2d(3.658, 4.506),
            // new Translation2d(4.489, 4.987),
            // new Translation2d(5.3213, 4.506),
            // new Translation2d(5.3213, 3.546),
            // new Translation2d(4.489, 3.065)
            // };
            // for (int i = 0; i < 6; i++) super.addBorderLine(reef_vortices[i],
            // reef_vortices[(i + 1) % 6]);

            // // the pillar in the middle of the field
            // super.addRectangularObstacle(0.305, 0.305, new Pose2d(8.774, 4.026, new
            // Rotation2d()));
        }
    }

    public ArenaSchool2025Reefscape() {
        super(new SchoolReefscapeFieldObstacleMap());
    }

    @Override public void placeGamePiecesOnField() {
        // Translation2d[] bluePositions = new Translation2d[] {
        // new Translation2d(1.219, 5.855), new Translation2d(1.219, 4.026), new
        // Translation2d(1.219, 2.197),
        // };
        // for (Translation2d position : bluePositions)
        // super.addGamePiece(new ReefscapeCoralAlgaeStack(position));

        // Translation2d[] redPositions = Arrays.stream(bluePositions)
        // .map(bluePosition -> new Translation2d(FieldMirroringUtils.FIELD_WIDTH -
        // bluePosition.getX(), bluePosition.getY()))
        // .toArray(Translation2d[]::new);
        // for (Translation2d position : redPositions)
        // super.addGamePiece(new ReefscapeCoralAlgaeStack(position));
    }

    @Override public void competitionPeriodic() {}

    @Override public synchronized List<Pose3d> getGamePiecesByType(String type) {
        List<Pose3d> poses = super.getGamePiecesByType(type);

        // add algae and coral stack
        // if (type.equals("Algae"))
        // poses.addAll(ReefscapeCoralAlgaeStack.getStackedAlgaePoses());
        // else if (type.equals("Coral"))
        // poses.addAll(ReefscapeCoralAlgaeStack.getStackedCoralPoses());

        return poses;
    }
}
