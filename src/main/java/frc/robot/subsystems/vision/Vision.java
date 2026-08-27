package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.vision.VisionConstants.AVG_DISTANCE_THRESHOLD_METERS;
import static frc.robot.subsystems.vision.VisionConstants.CAMERAS;
import static frc.robot.subsystems.vision.VisionConstants.NO_ODOMETRY_STD_MULTIPLIER;
import static frc.robot.subsystems.vision.VisionConstants.TRUST_LEVEL_MULTIPLIER;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.poseestimation.PoseEstimator;
import frc.robot.subsystems.poseestimation.PoseEstimator.VisionMeasurement;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.io.VisionAprilTagsIO;
import frc.robot.subsystems.vision.io.VisionAprilTagsIOPhoton;
import frc.robot.subsystems.vision.io.VisionAprilTagsIO.VisionData;
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
      VisionAprilTagsIO io = visionCameras[i];
      alertsGroup.addWarningAlert(() -> camera.name() + " Disconnected!", () -> !io.isConnected.getAsBoolean());
    }
  }

  private static List<VisionMeasurement> getAllResultsInIO(VisionAprilTagsIO io) {
    VisionData[] visionDataArr = io.visionData.get();
    List<VisionMeasurement> visionMesurments = new ArrayList<>();
    double stdFactor = io.getCameraConfig().stdFactor();
    for (VisionData visionData : visionDataArr) {
      int tagsUsed = visionData.tagsPoses().length;
      if (tagsUsed == 0) continue;
      if (visionData.ambiguity() > AMBIGUITY_THRESHOLD) continue;
      if (!FieldConstants.isOnField(visionData.robotPose())) continue;
      double distanceSum = 0;
      for (double distance : visionData.tagsDistancesToCam()) {
        distanceSum += distance;
      }
      double avgDistance = distanceSum / tagsUsed;
      if (avgDistance > AVG_DISTANCE_THRESHOLD_METERS) continue;
      TrustLevel trustLevels = calculateTrustLevel(stdFactor, tagsUsed, avgDistance, visionData.ambiguity());
      visionMesurments.add(new VisionMeasurement(visionData.robotPose().toPose2d(), trustLevels, visionData.timestamp()));
    }
    return visionMesurments;
  }

  private List<VisionMeasurement> getAllResults() {
    List<VisionMeasurement> measurments = new ArrayList<>();
    for (VisionAprilTagsIO io : visionCameras) {
      for (VisionMeasurement measurment : getAllResultsInIO(io)) {
        measurments.add(measurment);
      }
    }
    fieldsTable.recordOutput("Vision measurments", measurments.toArray(new VisionMeasurement[0]));
    return measurments;
  }

  public void update() {
    for (VisionMeasurement mesurment : getAllResults()) {
      PoseEstimator.getInstance().addVisionMeasurment(mesurment);
    }
  }

  public record TrustLevel(double xyStdDev, double rotationStdDev) { }

  private static TrustLevel calculateTrustLevel(double stdFactor, int tagsUsed, double avgDistanceToCam, double ambiguity) {
    if (ambiguity == 1 || tagsUsed == 0 || avgDistanceToCam == 0)
      return new TrustLevel(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    double value = Math.pow(avgDistanceToCam, 1.2) / Math.pow(tagsUsed, 2) / Math.pow(1 - ambiguity, 2) * stdFactor;
    double xyStdDev = TRUST_LEVEL_MULTIPLIER.xyStdDev * value;
    double rotationStdDevs = TRUST_LEVEL_MULTIPLIER.rotationStdDev * value;
    if (PoseEstimator.getInstance().inCollision() || DriverStation.isDisabled()) {
      xyStdDev *= NO_ODOMETRY_STD_MULTIPLIER;
      rotationStdDevs *= NO_ODOMETRY_STD_MULTIPLIER;
    }
    return new TrustLevel(xyStdDev, rotationStdDevs);
  }
}
