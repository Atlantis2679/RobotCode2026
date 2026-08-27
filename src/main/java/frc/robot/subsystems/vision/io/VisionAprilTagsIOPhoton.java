package frc.robot.subsystems.vision.io;

import static frc.robot.subsystems.vision.VisionConstants.APRTIL_TAGS_FIELD_LAYOUT;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Robot;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.VisionConstants.Sim;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class VisionAprilTagsIOPhoton extends VisionAprilTagsIO {
    private final PhotonCamera camera;
    private List<PhotonPipelineResult> photonPipelineResults;
    private final CameraConfig cameraConfig;
    private VisionData[] visionData;

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
        List<VisionData> visionData = new ArrayList<>();
        for (int i = 0; i < photonPipelineResults.size(); i++) {
            PhotonPipelineResult result = photonPipelineResults.get(i);
            if (result.hasTargets()) {
                if (result.getMultiTagResult().isPresent()) {
                    Transform3d cameraToPose = result.multitagResult.get().estimatedPose.best;
                    Pose3d robotPose = new Pose3d().transformBy(cameraToPose).transformBy(cameraConfig.robotToCam().inverse());
                    double timestamp = result.getTimestampSeconds();
                    Pose3d[] targetsPoses = result.multitagResult.get().fiducialIDsUsed.stream()
                            .map(targetId -> APRTIL_TAGS_FIELD_LAYOUT.getTagPose(targetId))
                            .flatMap(Optional::stream).toList().toArray(new Pose3d[0]);
                    double[] tagsDistancesToCam = new double[result.targets.size()];
                    for (int j = 0; j < tagsDistancesToCam.length; ++j) {
                        tagsDistancesToCam[j] = result.targets.get(j).getBestCameraToTarget().getTranslation()
                                .getNorm();
                    }
                    visionData.add(new VisionData(timestamp, robotPose, targetsPoses, 0, tagsDistancesToCam));
                } else {
                    PhotonTrackedTarget bestTarget = result.getBestTarget();
                    if (APRTIL_TAGS_FIELD_LAYOUT.getTagPose(bestTarget.fiducialId).isEmpty()) continue;
                    Pose3d tagPose = APRTIL_TAGS_FIELD_LAYOUT.getTagPose(bestTarget.fiducialId).get();
                    Transform3d camToTarget = bestTarget.bestCameraToTarget;
                    Transform3d robotToTarget = cameraConfig.robotToCam().plus(camToTarget);
                    Pose3d robotPose = tagPose.transformBy(robotToTarget.inverse());
                    double timestamp = result.getTimestampSeconds();
                    visionData.add(new VisionData(timestamp, robotPose, new Pose3d[] { tagPose }, bestTarget.poseAmbiguity,
                        new double[] { camToTarget.getTranslation().getNorm() }));
                }
            }
        }
        this.visionData = visionData.toArray(new VisionData[0]);
    }

    @Override
    public CameraConfig getCameraConfig() {
        return cameraConfig;
    }

    public VisionData[] visionData() {
        return visionData;
    }

    @Override
    protected boolean getIsConnected() {
        return camera.isConnected();
    }

}
