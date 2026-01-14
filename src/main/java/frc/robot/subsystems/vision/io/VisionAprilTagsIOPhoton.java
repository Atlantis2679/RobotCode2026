package frc.robot.subsystems.vision.io;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class VisionAprilTagsIOPhoton extends VisionAprilTagsIO {
    private final PhotonCamera camera;
    private List<PhotonPipelineResult> photonPipelineResults;
    // private List<EstimatedRobotPose> photonEstimatorResults = new ArrayList<>();
    private final CameraConfig cameraConfig;

    public VisionAprilTagsIOPhoton(LogFieldsTable fieldsTable, CameraConfig cameraConfig) {
        super(fieldsTable.getSubTable(cameraConfig.name()));

        this.camera = new PhotonCamera(cameraConfig.name());

        if (Robot.isSimulation()) {
            PhotonCameraSim photonCameraSim = new PhotonCameraSim(camera, Sim.SIM_CAMERA_PROPERTIES);
            Sim.VISION_SIM.addCamera(photonCameraSim, cameraConfig.robotToCam());
        }

        this.cameraConfig = cameraConfig;
    }

    @Override
    public void periodicBeforeFields() {
        photonPipelineResults = camera.getAllUnreadResults();
    }
    
    @Override
    public CameraConfig getCameraConfig() {
        return cameraConfig;
    }

    @Override
    protected double[] getCameraTimestampsSeconds() {
        double[] timestamps = new double[photonPipelineResults.size()];

        for (int i = 0; i < timestamps.length; i++) {
            timestamps[i] = photonPipelineResults.get(i).getTimestampSeconds();
        }
        return timestamps;
    }

    @Override
    protected Pose3d[] getRobotPoses() {
        Pose3d[] robotPoses = new Pose3d[photonPipelineResults.size()];
        int count = 0;
        for (int i = 0; i < robotPoses.length; i++) {
            PhotonPipelineResult result = photonPipelineResults.get(i);
            if (result.hasTargets()) {
                if (result.getMultiTagResult().isPresent()) {
                    Transform3d cameraToPose = result.multitagResult.get().estimatedPose.best;
                    Pose3d robotPose = new Pose3d().transformBy(cameraToPose).transformBy(cameraConfig.robotToCam())
                        .relativeTo(APRTIL_TAGS_FIELD_LAYOUT.getOrigin());
                    robotPoses[i] = robotPose;
                } else {
                    PhotonTrackedTarget target = result.getTargets().get(0);
                    Pose3d tagPose = APRTIL_TAGS_FIELD_LAYOUT.getTagPose(target.fiducialId).get(); // Crash if tag not on field!
                    Transform3d targetToCam = target.bestCameraToTarget.inverse();
                    Pose3d robotPose = tagPose.transformBy(targetToCam);
                    robotPoses[i] = robotPose;
                }
                count++;
            }
        }
        return Arrays.copyOf(robotPoses, count);
    }

    @Override
    protected Pose3d[][] getTagsPoses() {
        Pose3d[][] tagsPoses = new Pose3d[photonPipelineResults.size()][];
        for (int i = 0; i < tagsPoses.length; i++) {
            tagsPoses[i] = new Pose3d[photonPipelineResults.get(i).targets.size()];
            for (int j = 0; j < tagsPoses[i].length; j++) {
                tagsPoses[i][j] = APRTIL_TAGS_FIELD_LAYOUT.getTagPose(i).get(); // Crash if tag not on field!
            }
        }
        return tagsPoses;
    }

    @Override
    protected double[][] getTagsDistanceToCam() {
        double[][] tagsDistanceToCam = new double[photonPipelineResults.size()][];
        for (int i = 0; i < tagsDistanceToCam.length; i++) {
            tagsDistanceToCam[i] = new double[photonPipelineResults.get(i).targets.size()];
            for (int j = 0; j < tagsDistanceToCam[i].length; j++) {
                PhotonTrackedTarget target = photonPipelineResults.get(i).targets.get(j);
                tagsDistanceToCam[i][j] = target.getBestCameraToTarget().getTranslation().getNorm(); // Crash if tag not on field!
            }
        }
        return tagsDistanceToCam;
    }

    @Override
    protected double[] getTagsAmbiguities() {
        double[] ambiguities = new double[photonPipelineResults.size()];
        for (int i = 0; i < ambiguities.length; i++) {
            if (photonPipelineResults.get(i).multitagResult.isPresent()) {
                ambiguities[i] = 0;
            } else {
                PhotonTrackedTarget target = photonPipelineResults.get(i).targets.get(0);
                ambiguities[i] = target.getPoseAmbiguity();
            }
        }
        return ambiguities;
    }

    @Override
    protected boolean getIsConnected() {
        return camera.isConnected();
    }
}
