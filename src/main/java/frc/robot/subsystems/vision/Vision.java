package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.AMBIGUITY_THREASHOLD;
import static frc.robot.subsystems.vision.VisionConstants.AVG_DISTANCE_THREASHOLD_METERS;
import static frc.robot.subsystems.vision.VisionConstants.CAMERAS;
import static frc.robot.subsystems.vision.VisionConstants.ROTATION_STD_MULTIPLYER;
import static frc.robot.subsystems.vision.VisionConstants.TRANSLATION_STD_MULTIPLYER;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.PoseEstimator.VisionMesurment;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.io.VisionAprilTagsIO;
import frc.robot.subsystems.vision.io.VisionAprilTagsIOPhoton;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

public class Vision {
  private final LogFieldsTable fieldsTable = new LogFieldsTable("Vision");
  private final VisionAprilTagsIO[] visionCameras = new VisionAprilTagsIO[CAMERAS.length];

  public Vision() {
    PeriodicAlertsGroup alertsGroup = new PeriodicAlertsGroup("VisionAlerts");
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
      double ambiguity = io.tagsAmbiguities.get()[i];
      if (ambiguity > AMBIGUITY_THREASHOLD) continue;
      if (!FieldConstants.isOnField(pose)) continue;
      double distanceSum = 0;
      for (double distance : io.tagsDistanceToCam.get()[i]) {
        distanceSum += distance;
      }
      double avgDistance = distanceSum / tagsUsed;
      if (avgDistance > AVG_DISTANCE_THREASHOLD_METERS) continue;
      double[] trustLevels = calculateTrustLevel(stdFactor, tagsUsed, avgDistance, ambiguity, useRoation);
      visionMesurments.add(new VisionMesurment(pose.toPose2d(), trustLevels[0], trustLevels[1], io.cameraTimestampsSeconds.get()[i]));
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
    fieldsTable.recordOutput("Vision measurments", measurments.toArray(new VisionMesurment[0]));
    return measurments;
  }

  private static double[] calculateTrustLevel(double stdFactor, int tagsUsed, double avgDistanceToCam, double ambiguity, boolean useRotation) {
    if (ambiguity == 1 || tagsUsed == 0 || avgDistanceToCam == 0)
      return new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
    double value = Math.pow(avgDistanceToCam, 1.2) / Math.pow(tagsUsed, 2) / Math.pow(1 - ambiguity, 2) * stdFactor;
    double xyStdDev = TRANSLATION_STD_MULTIPLYER * value;
    double rotationStdDevs = useRotation ? ROTATION_STD_MULTIPLYER * value : Double.POSITIVE_INFINITY;
    return new double[] {xyStdDev, rotationStdDevs};
  }
}
