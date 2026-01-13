package frc.robot.subsystems.vision.io;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose3d;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class VisionAprilTagsIOPhoton extends VisionAprilTagsIO {
    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera camera;
    private List<PhotonPipelineResult> photonPipelineResults;
    private List<EstimatedRobotPose> photonEstimatorResults = new ArrayList<>();
    private final CameraConfig cameraConfig;

    public VisionAprilTagsIOPhoton(LogFieldsTable fieldsTable, CameraConfig cameraConfig) {
        super(fieldsTable.getSubTable(cameraConfig.name()));

        this.camera = new PhotonCamera(cameraConfig.name());

        this.cameraConfig = cameraConfig;
      
        photonPoseEstimator = new PhotonPoseEstimator(APRTIL_TAGS_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            cameraConfig.robotToCam());
    }

    @Override
    public void periodicBeforeFields() {
        photonPipelineResults = camera.getAllUnreadResults();
        for (PhotonPipelineResult photonPipelineResult : photonPipelineResults) {
            Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(photonPipelineResult);
            photonEstimatorResults = new ArrayList<>();
            if (estimatedPose.isPresent()) {
                photonEstimatorResults.add(estimatedPose.get());
            }
        }
    }

    @Override
    public CameraConfig getCameraConfig() {
      return cameraConfig;
    }

    @Override
    protected double[] getCameraTimestampsSeconds() {
        double[] timestamps = new double[photonEstimatorResults.size()];

        for (int i = 0; i < timestamps.length; i++) {
            timestamps[i] = photonEstimatorResults.get(i).timestampSeconds;
        }
        return timestamps;
    }

    @Override
    protected Pose3d[] getRobotPoses() {
        Pose3d[] robotPoses = new Pose3d[photonEstimatorResults.size()];
        for (int i = 0; i < robotPoses.length; i++) {
            robotPoses[i] = photonEstimatorResults.get(i).estimatedPose.transformBy(cameraConfig.robotToCam().inverse());
        }
        return robotPoses;
    }

    @Override
    protected Pose3d[][] getTagsPoses() {
        Pose3d[][] tagsPoses = new Pose3d[photonEstimatorResults.size()][];
        for (int i = 0; i < tagsPoses.length; i++) {
            tagsPoses[i] = new Pose3d[photonEstimatorResults.get(i).targetsUsed.size()];
            for (int j = 0; j < tagsPoses[i].length; j++) {
                tagsPoses[i][j] = APRTIL_TAGS_FIELD_LAYOUT.getTagPose(photonEstimatorResults.get(i).targetsUsed.get(j).fiducialId).get(); // Crash if not present
            }
        }
        return tagsPoses;
    }

    @Override
    protected double[][] getTagsDistanceToCam() {
        double[][] tagsDistanceToCam = new double[photonEstimatorResults.size()][];
        for (int i = 0; i < tagsDistanceToCam.length; i++) {
            tagsDistanceToCam[i] = new double[photonEstimatorResults.get(i).targetsUsed.size()];
            for (int j = 0; j < tagsDistanceToCam[i].length; j++) {
                tagsDistanceToCam[i][j] = photonEstimatorResults.get(i).targetsUsed.get(j).bestCameraToTarget.getTranslation().getNorm();
            }
        }
        return tagsDistanceToCam;
    }

    @Override
    protected double[][] getTagsAmbiguities() {
        double[][] ambiguities = new double[photonEstimatorResults.size()][];
        for (int i = 0; i < ambiguities.length; i++) {
            ambiguities[i] = new double[photonEstimatorResults.get(i).targetsUsed.size()];
            for (int j = 0; j < ambiguities[i].length; j++) {
                ambiguities[i][j] = photonEstimatorResults.get(i).targetsUsed.get(j).poseAmbiguity;
            }
        }
        return ambiguities;
    }

    @Override
    protected boolean getIsConnected() {
        return camera.isConnected();
    }
}
