package frc.robot.subsystems.vision;


import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

public interface VisionIO {
    public static class VisionIOInputs implements LoggableInputs {
        public boolean camera_connected = false;
        public boolean driver_mode = false;
        public int pipeline_index = 0;
        public PhotonPipelineResult[] photon_results = new PhotonPipelineResult[0];
        public Optional<Matrix<N3, N3>> camera_matrix_opt = Optional.empty();
        public Optional<Matrix<N8, N1>> dist_coeffs_opt = Optional.empty();

        public void toLog(LogTable table) {
            table.put("Camera_connected", camera_connected);
            table.put("Driver_mode", driver_mode);
            table.put("Pipeline_index", pipeline_index);

            PhotonLoggingUtils.cameraMatrixOptToLog(table, camera_matrix_opt);
            PhotonLoggingUtils.distCoeffsOptToLog(table, dist_coeffs_opt);
            PhotonLoggingUtils.pipelineResultsToLog(table, photon_results);
        }

        public void fromLog(LogTable table) {
            camera_connected = table.get("Camera_connected", camera_connected);
            driver_mode = table.get("Driver_mode", driver_mode);
            pipeline_index = table.get("Pipeline_index", pipeline_index);

            photon_results = PhotonLoggingUtils.pipelineResultsFromLog(table);
            camera_matrix_opt = PhotonLoggingUtils.cameraMatrixOptFromLog(table);
            dist_coeffs_opt = PhotonLoggingUtils.distCoeffsOptFromLog(table);
        }
    }

    public default void updateInputs(final VisionIOInputs inputs) {}
}
