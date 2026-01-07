package frc.robot.subsystems.swerve.vision;

import static frc.robot.subsystems.swerve.vision.VisionConstants.AMBIGUITY_THREASHOLD;
import static frc.robot.subsystems.swerve.vision.VisionConstants.AVG_DIUSTANCE_DEGREDATION_START_METERS;
import static frc.robot.subsystems.swerve.vision.VisionConstants.CAMERAS;
import static frc.robot.subsystems.swerve.vision.VisionConstants.ROTATION_STD_MULTIPLYER;
import static frc.robot.subsystems.swerve.vision.VisionConstants.TRANSLATION_STD_MULTIPLYER;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.PoseEstimator.VisionMesurment;
import frc.robot.subsystems.swerve.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.swerve.vision.io.VisionAprilTagsIO;
import frc.robot.subsystems.swerve.vision.io.VisionAprilTagsIOPhoton;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class Vision {
  private final LogFieldsTable fieldsTable = new LogFieldsTable("Vision");
  private final VisionAprilTagsIO[] visionCameras = new VisionAprilTagsIO[CAMERAS.length];

  public Vision() {
    PeriodicAlertsGroup alertsGroup = new PeriodicAlertsGroup("Vision");
    for (int i = 0; i < visionCameras.length; i++) {
      CameraConfig camera = CAMERAS[i];
      visionCameras[i] = new VisionAprilTagsIOPhoton(fieldsTable, camera);
      alertsGroup.addWarningAlert(() -> camera.name(), visionCameras[i].isConnected);
    }
  }

  private static List<VisionMesurment> getAllResultsInIO(VisionAprilTagsIO io, boolean useRoation) {
    int length = io.cameraTimestampsSeconds.get().length;
    List<VisionMesurment> visionMesurments = new ArrayList<>();
    double stdFactor = io.getCameraConfig().stdFactor();
    for (int i = 0; i < length; i++) {
      int tagsUsed = io.tagsPoses.get()[i].length;
      Pose3d pose = io.posesEstimates.get()[i];
      double maxAmbiguity = 0;
      for (double ambiguities : io.tagsAmbiguities.get()[i]) {
        maxAmbiguity = Math.max(maxAmbiguity, ambiguities);
      }
      if (maxAmbiguity > AMBIGUITY_THREASHOLD) continue;
      if (!FieldConstants.isOnField(pose)) continue;
      double distanceSum = 0;
      for (double distance : io.tagsDistanceToCam.get()[i]) {
        distanceSum += distance;
      }
      double avgDistance = distanceSum / tagsUsed;
      Vector<N3> trustLevels = calculateTrustLevel(stdFactor, tagsUsed, avgDistance, maxAmbiguity, useRoation);
      visionMesurments.add(new VisionMesurment(pose.toPose2d(), trustLevels, io.cameraTimestampsSeconds.get()[i]));
    }
    return visionMesurments;
  }

  public List<VisionMesurment> getAllResults(boolean useRotation) {
    List<VisionMesurment> measurments = new ArrayList<>();
    for (VisionAprilTagsIO io : visionCameras) {
      for (VisionMesurment measurment : getAllResultsInIO(io, useRotation)) {
        measurments.add(measurment);
      }
    }
    return measurments;
  }

  private static Vector<N3> calculateTrustLevel(double stdFactor, int tagsUsed, double avgDistanceToCam, double maxAmbiguity, boolean useRotation) {
    if (maxAmbiguity == 1 || tagsUsed == 0 || avgDistanceToCam == 0)
      return VecBuilder.fill(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    double value = Math.pow(avgDistanceToCam + AVG_DIUSTANCE_DEGREDATION_START_METERS, 1.2) / Math.pow(tagsUsed, 2) / Math.pow(1 - maxAmbiguity, 2) * stdFactor;
    double xyStdDev = TRANSLATION_STD_MULTIPLYER * value;
    double rotationStdDevs = useRotation ? ROTATION_STD_MULTIPLYER * value : Double.POSITIVE_INFINITY;
    return VecBuilder.fill(xyStdDev, xyStdDev, rotationStdDevs);
  }
}
