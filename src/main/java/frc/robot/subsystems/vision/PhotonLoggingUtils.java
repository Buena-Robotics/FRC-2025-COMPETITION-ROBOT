package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.LogTable;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N8;

public class PhotonLoggingUtils {
    public static void pipelineResultsToLog(LogTable table, PhotonPipelineResult[] results){
        table.put("Photon_results/length", results.length);
        for (int i = 0; i < results.length; i++) {
            table.put("Photon_results/" + i, results[i]);
        }
    }
    public static PhotonPipelineResult[] pipelineResultsFromLog(LogTable table){
        int results_length = table.get("Photon_results/length", 0);
        if(results_length == 0) return new PhotonPipelineResult[0];
        PhotonPipelineResult[] results = new PhotonPipelineResult[results_length];
        for (int i = 0; i < results_length; i++) {
            results[i] = table.get("Photon_results/" + i, new PhotonPipelineResult());
        }
        return results;
    }
    public static void cameraMatrixOptToLog(LogTable table, Optional<Matrix<N3, N3>> camera_matrix_opt){
        if(camera_matrix_opt.isPresent())
            table.put("Camera_matrix_opt", camera_matrix_opt.get());
    }
    public static Optional<Matrix<N3, N3>> cameraMatrixOptFromLog(LogTable table){
        Matrix<N3, N3> default_value = new Matrix<N3, N3>(Nat.N3(),Nat.N3());
        Matrix<N3, N3> result = table.get("Camera_matrix_opt", default_value);
        return result == default_value ? Optional.empty() : Optional.of(result);
    }
    public static void distCoeffsOptToLog(LogTable table, Optional<Matrix<N8, N1>> dist_coeffs_opt){
        if(dist_coeffs_opt.isPresent())
            table.put("Dist_coeffs_opt", dist_coeffs_opt.get());
    }
    public static Optional<Matrix<N8, N1>> distCoeffsOptFromLog(LogTable table){
        Matrix<N8, N1> default_value = new Matrix<N8, N1>(Nat.N8(),Nat.N1());
        Matrix<N8, N1> result = table.get("Dist_coeffs_opt", default_value);
        return result == default_value ? Optional.empty() : Optional.of(result);
    }
}
