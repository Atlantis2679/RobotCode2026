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
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
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
                new Transform3d());
    }

    @Override
    public void periodicBeforeFields() {
        photonPipelineResults = camera.getAllUnreadResults();
        photonEstimatorResults = new ArrayList<>();
        for (PhotonPipelineResult pipelineResult : photonPipelineResults) {
            Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(pipelineResult);
            if (pose.isPresent()) {
                photonEstimatorResults.add(pose.get());
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
            robotPoses[i] = photonEstimatorResults.get(i).estimatedPose;
        }
        return robotPoses;
    }

    @Override
    protected Pose3d[][] getTagsPoses() {
        Pose3d[][] tagsPoses = new Pose3d[photonEstimatorResults.size()][];
        for (int i = 0; i < photonEstimatorResults.size(); i++) {
            List<PhotonTrackedTarget> targets = photonEstimatorResults.get(i).targetsUsed;
            tagsPoses[i] = new Pose3d[targets.size()];
            for (int j = 0; j < targets.size(); j++) {
                tagsPoses[i][j] = APRTIL_TAGS_FIELD_LAYOUT.getTagPose(targets.get(j).fiducialId).orElse(new Pose3d());
            }
        }
        return tagsPoses;
    }

    @Override
    protected double[][] getTagsDistanceToCam() {
        double[][] tagsDistanceToCam = new double[photonEstimatorResults.size()][];
        for (int i = 0; i < photonEstimatorResults.size(); i++) {
            List<PhotonTrackedTarget> targets = photonEstimatorResults.get(i).targetsUsed;
            tagsDistanceToCam[i] = new double[targets.size()];
            for (int j = 0; j < targets.size(); j++) {
                tagsDistanceToCam[i][j] = targets.get(j).bestCameraToTarget.getTranslation().getNorm();
            }
        }
        return tagsDistanceToCam;
    }

    @Override
    protected double[][] getTagsAmbiguities() {
        double[][] ambiguities = new double[photonEstimatorResults.size()][];
        for (int i = 0; i < photonEstimatorResults.size(); i++) {
            List<PhotonTrackedTarget> targets = photonEstimatorResults.get(i).targetsUsed;
            ambiguities[i] = new double[targets.size()];
            for (int j = 0; j < targets.size(); j++) {
                ambiguities[i][j] = targets.get(j).getPoseAmbiguity();
            }
        }
        return ambiguities;
    }

    @Override
    protected boolean getIsConnected() {
        return camera.isConnected();
    }
}
