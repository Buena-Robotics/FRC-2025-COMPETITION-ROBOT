package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.photonvision.estimation.CameraTargetRelation;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.subsystems.vision.Cameras.Camera;

public class VisionIOPhotonServo extends VisionIOPhoton {
    private static final double MAX_SIGHT_RANGE_METERS = 6.0;
    private static VisionTargetSim[] targets;

    // private final Transform3d servo_camera_offset;
    protected final SillyServo servo;

    public VisionIOPhotonServo(final Camera camera_info, final Supplier<Pose2d> robot_pose_supplier, final int servo_channel, final double servo_radius_cm) {
        super(camera_info, robot_pose_supplier);
        this.servo = new SillyServo(servo_channel);
        // this.servo_camera_offset = new Transform3d(servo_radius_cm / 100.0, 0.0, 0.0,
        // new Rotation3d());

        this.servo.setAngle(90.0);

        if (targets == null) {
            final List<AprilTag> apriltags = FieldConstants.APRILTAG_LAYOUT.getTags();
            targets = new VisionTargetSim[apriltags.size()];
            for (int i = 0; i < targets.length; i++) {
                targets[i] = new VisionTargetSim(apriltags.get(i).pose, TargetModel.kAprilTag36h11, apriltags.get(i).ID);
            }
        }
        super.setCameraTransformSupplier(this::getCameraTransform);
    }

    @Override public void updateInputs(VisionIOInputs inputs) {
        servo.periodic();

        final double best_angle = bestCloseServoAngle(servo.getGuessedAngle());
        Logger.recordOutput("Vision/Cameras/" + camera_info.name() + "/Pose", getCameraPose());
        Logger.recordOutput("Vision/Cameras/" + camera_info.name() + "/BestAngle", best_angle);
        Logger.recordOutput("Vision/Cameras/" + camera_info.name() + "/TagsSpotted", apriltagsSpotted(servo.getAngle()));
        Logger.recordOutput("Vision/Cameras/" + camera_info.name() + "/GuessedAngle", servo.getGuessedAngle());
        Logger.recordOutput("Vision/Cameras/" + camera_info.name() + "/SetAngle", servo.getAngle());

        super.updateInputs(inputs);
    }

    protected int apriltagsSpotted(final double servo_angle) {
        final Pose3d camera_pose = getCameraPose(servo_angle);
        int count = 0;
        for (VisionTargetSim target : targets) {
            if (canSeeTargetPose(camera_pose, target))
                count++;
        }
        return count;
    }

    final double[] dummy_angle_offsets = { 2.0, 4.0, 8.0, 22.0, 45.0, 60.0, 75.0, 90.0, 120.0, 150.0, 170.0 };

    protected double bestCloseServoAngle(final double servo_angle) {
        double best_angle = servo_angle;
        int best_count = apriltagsSpotted(servo_angle);
        for (int i = 0; i < dummy_angle_offsets.length; i++) {
            if (servo_angle + dummy_angle_offsets[i] <= 0 || servo_angle + dummy_angle_offsets[i] >= 180)
                continue;
            int count = apriltagsSpotted(servo_angle + dummy_angle_offsets[i]);
            if (count > best_count) {
                best_count = count;
                best_angle = servo_angle + dummy_angle_offsets[i];
                break;
            }
            if (servo_angle - dummy_angle_offsets[i] <= 0 || servo_angle - dummy_angle_offsets[i] >= 180)
                continue;
            count = apriltagsSpotted(servo_angle - dummy_angle_offsets[i]);
            if (count > best_count) {
                best_count = count;
                best_angle = servo_angle - dummy_angle_offsets[i];
                break;
            }
        }
        servo.setAngle(best_angle);
        return best_angle;
    }

    protected boolean canSeeTargetPose(final Pose3d cam_pose, final VisionTargetSim target) {
        final CameraTargetRelation relation = new CameraTargetRelation(cam_pose, target.getPose());
        return (
        // target translation is outside of camera's FOV
        (Math.abs(relation.camToTargYaw.getDegrees()) < camera_info.camera_properties().getHorizFOV().getDegrees() / 2) && (Math.abs(relation.camToTargPitch.getDegrees()) < camera_info.camera_properties().getVertFOV().getDegrees() / 2) &&
            (!target.getModel().isPlanar || Math.abs(relation.targToCamAngle.getDegrees()) < 90) // camera is behind planar target and it should be occluded
            && (relation.camToTarg.getTranslation().getNorm() <= MAX_SIGHT_RANGE_METERS)); // target is too far
    }

    @Override public Pose3d getCameraPose() {
        return getCameraPose(servo.getGuessedAngle());
    }

    public Pose3d getCameraPose(final double dummy_servo_angle) {
        return super.getCameraPose().transformBy(new Transform3d(0, 0, 0, new Rotation3d(0.0, 0.0, Units.degreesToRadians(dummy_servo_angle - 90.0))));
    }

    public Transform3d getCameraTransform() {
        return new Transform3d(0, 0, 0, new Rotation3d(0.0, 0.0, Units.degreesToRadians(servo.getGuessedAngle() - 90.0)));
    }
}
