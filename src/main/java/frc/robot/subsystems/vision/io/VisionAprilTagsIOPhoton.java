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
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class VisionAprilTagsIOPhoton extends VisionAprilTagsIO {
    private final PhotonCamera camera;
    private final CameraConfig cameraConfig;
    private VisionData[] visionData;
    private boolean moreThanOneTargetInSingleTargetMode = false;

    public VisionAprilTagsIOPhoton(LogFieldsTable fieldsTable, CameraConfig cameraConfig) {
        super(fieldsTable.getSubTable(cameraConfig.name()));

        this.camera = new PhotonCamera(cameraConfig.name());

        if (Robot.isSimulation()) {
            PhotonCameraSim photonCameraSim = new PhotonCameraSim(camera, cameraConfig.simCameraProperties());
            Sim.VISION_SIM.addCamera(photonCameraSim, cameraConfig.robotToCam());
        }

        new PeriodicAlertsGroup("Vision").addWarningAlert(() -> "PhotonVision multitag is failed or is disabled", () -> this.moreThanOneTargetInSingleTargetMode);

        this.cameraConfig = cameraConfig;
    }

    @Override
    public void periodicBeforeFields() {
        List<PhotonPipelineResult> photonPipelineResults = camera.getAllUnreadResults();
        List<VisionData> visionData = new ArrayList<>();
        List<Pose3d[]> tagsPosesList = new ArrayList<>();
        for (int i = 0; i < photonPipelineResults.size(); i++) {
            PhotonPipelineResult result = photonPipelineResults.get(i);
            if (result.hasTargets()) {
                if (result.getMultiTagResult().isPresent()) {
                    Transform3d cameraToPose = result.multitagResult.get().estimatedPose.best;
                    Pose3d robotPose = new Pose3d().transformBy(cameraToPose).transformBy(cameraConfig.robotToCam().inverse());
                    double timestamp = result.getTimestampSeconds();
                    Pose3d[] tagsPoses = result.multitagResult.get().fiducialIDsUsed.stream()
                            .map(targetId -> APRTIL_TAGS_FIELD_LAYOUT.getTagPose(targetId))
                            .flatMap(Optional::stream).toList().toArray(new Pose3d[0]);
                    int tagsUsed = tagsPoses.length;
                    if (tagsUsed == 0) continue;
                    double distanceSum = 0;
                    for (Pose3d target : tagsPoses) {
                      distanceSum += target.relativeTo(robotPose).getTranslation().getNorm();
                    }
                    tagsPosesList.add(tagsPoses);
                    visionData.add(new VisionData(timestamp, robotPose, distanceSum / tagsUsed, result.multitagResult.get().estimatedPose.ambiguity, tagsUsed));
                } else {
                    moreThanOneTargetInSingleTargetMode = result.getTargets().size() > 1;
                    PhotonTrackedTarget target = result.getBestTarget();
                    if (APRTIL_TAGS_FIELD_LAYOUT.getTagPose(target.fiducialId).isEmpty()) continue;
                    Pose3d tagPose = APRTIL_TAGS_FIELD_LAYOUT.getTagPose(target.fiducialId).get();
                    tagsPosesList.add(new Pose3d[] { tagPose });
                    Transform3d camToTarget = target.bestCameraToTarget;
                    Transform3d robotToTarget = cameraConfig.robotToCam().plus(camToTarget);
                    Pose3d robotPose = tagPose.transformBy(robotToTarget.inverse());
                    double camToTargetDistance = robotToTarget.getTranslation().getNorm();
                    double timestamp = result.getTimestampSeconds();
                    visionData.add(new VisionData(timestamp, robotPose, camToTargetDistance, target.poseAmbiguity, 1));
                }
            }
        }
        fields.recordOutput("tagsPoses", tagsPosesList.toArray(new Pose3d[0][]));
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
