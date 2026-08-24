package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.vision.VisionConstants.AVG_DISTANCE_THRESHOLD_METERS;
import static frc.robot.subsystems.vision.VisionConstants.CAMERAS;
import static frc.robot.subsystems.vision.VisionConstants.NO_ODOMETRY_STD_MULTIPLIER;
import static frc.robot.subsystems.vision.VisionConstants.TRUST_LEVEL_MULTIPLIER;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.poseestimation.PoseEstimator;
import frc.robot.subsystems.poseestimation.PoseEstimator.VisionMeasurement;
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
      VisionAprilTagsIO io = visionCameras[i];
      alertsGroup.addWarningAlert(() -> camera.name() + " Disconnected!", () -> !io.isConnected.getAsBoolean());
    }
  }

  private static List<VisionMeasurement> getAllResultsInIO(VisionAprilTagsIO io) {
    int length = io.posesEstimates.get().length;
    List<VisionMeasurement> visionMesurments = new ArrayList<>();
    double stdFactor = io.getCameraConfig().stdFactor();
    for (int i = 0; i < length; i++) {
      int tagsUsed = io.tagsPoses.get()[i].length;
      if (tagsUsed == 0) continue;
      Pose3d pose = io.posesEstimates.get()[i];
      double ambiguity = io.tagsAmbiguities.get()[i];
      if (ambiguity > AMBIGUITY_THRESHOLD) continue;
      if (!FieldConstants.isOnField(pose)) continue;
      double distanceSum = 0;
      for (double distance : io.tagsDistanceToCam.get()[i]) {
        distanceSum += distance;
      }
      double avgDistance = distanceSum / tagsUsed;
      if (avgDistance > AVG_DISTANCE_THRESHOLD_METERS) continue;
      TrustLevel trustLevels = calculateTrustLevel(stdFactor, tagsUsed, avgDistance, ambiguity);
      visionMesurments.add(new VisionMeasurement(pose.toPose2d(), trustLevels, io.cameraTimestampsSeconds.get()[i]));
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
