package frc.robot.subsystems.vision.io;

import static frc.robot.subsystems.vision.VisionConstants.*;

import java.util.ArrayList;
import java.util.List;

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
    public record VisionData(double timestamp, Pose3d robotPose, Pose3d[] tagsPoses, double ambiguity,
            double[] tagsDistancesToCam) {
    }

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
                    // Transform3d robotToPose = cameraToPose.plus(cameraConfig.robotToCam());
                    Pose3d robotPose = new Pose3d().transformBy(cameraToPose).transformBy(cameraConfig.robotToCam().inverse());
                    double timestamp = result.getTimestampSeconds();
                    Pose3d[] targetsPoses = result.targets.stream()
                            .map(target -> APRTIL_TAGS_FIELD_LAYOUT.getTagPose(target.fiducialId).get()).toList()
                            .toArray(new Pose3d[0]);
                    double[] tagsDistancesToCam = new double[result.targets.size()];
                    for (int j = 0; j < tagsDistancesToCam.length; ++j) {
                        tagsDistancesToCam[j] = result.targets.get(j).getBestCameraToTarget().getTranslation()
                                .getNorm();
                    }
                    visionData.add(new VisionData(timestamp, robotPose, targetsPoses, 0, tagsDistancesToCam));
                } else {
                    PhotonTrackedTarget target = result.getTargets().get(0);
                    Pose3d tagPose = APRTIL_TAGS_FIELD_LAYOUT.getTagPose(target.fiducialId).get(); // Crash if tag not
                                                                                                   // on field!
                    Transform3d camToTarget = target.bestCameraToTarget;
                    Transform3d robotToTarget = cameraConfig.robotToCam().plus(camToTarget);
                    Pose3d robotPose = tagPose.transformBy(robotToTarget.inverse());
                    double timestamp = result.getTimestampSeconds();
                    visionData.add(new VisionData(timestamp, robotPose, new Pose3d[] { tagPose }, target.poseAmbiguity,
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

    @Override
    protected double[] getCameraTimestampsSeconds() {
        double[] timestamps = new double[visionData.length];
        for (int i = 0; i < timestamps.length; i++) {
            timestamps[i] = visionData[i].timestamp;
        }
        return timestamps;
    }

    @Override
    protected Pose3d[] getRobotPoses() {
        Pose3d[] robotPoses = new Pose3d[visionData.length];
        for (int i = 0; i < robotPoses.length; i++) {
            robotPoses[i] = visionData[i].robotPose;
        }
        return robotPoses;
    }

    @Override
    protected Pose3d[][] getTagsPoses() {
        Pose3d[][] tagPoses = new Pose3d[visionData.length][];
        for (int i = 0; i < tagPoses.length; i++) {
            tagPoses[i] = visionData[i].tagsPoses;
        }
        return tagPoses;
    }

    @Override
    protected double[][] getTagsDistanceToCam() {
        double[][] distances = new double[visionData.length][];
        for (int i = 0; i < distances.length; i++) {
            distances[i] = visionData[i].tagsDistancesToCam;
        }
        return distances;
    }

    @Override
    protected double[] getTagsAmbiguities() {
        double[] ambiguities = new double[visionData.length];
        for (int i = 0; i < ambiguities.length; i++) {
            ambiguities[i] = visionData[i].ambiguity;
        }
        return ambiguities;
    }

    @Override
    protected boolean getIsConnected() {
        return camera.isConnected();
    }

}
