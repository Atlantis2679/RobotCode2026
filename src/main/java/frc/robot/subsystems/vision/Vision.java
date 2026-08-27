package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.vision.VisionConstants.AVG_DISTANCE_THRESHOLD_METERS;
import static frc.robot.subsystems.vision.VisionConstants.CAMERAS;
import static frc.robot.subsystems.vision.VisionConstants.NO_ODOMETRY_TRUST_LEVEL_MULTIPLIER;
import static frc.robot.subsystems.vision.VisionConstants.TRUST_LEVEL_MULTIPLIER;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;
import frc.robot.subsystems.poseestimation.PoseEstimator;
import frc.robot.subsystems.poseestimation.PoseEstimator.VisionMeasurement;
import frc.robot.subsystems.vision.VisionConstants.CameraConfig;
import frc.robot.subsystems.vision.io.VisionAprilTagsIO;
import frc.robot.subsystems.vision.io.VisionAprilTagsIO.VisionData;
import frc.robot.subsystems.vision.io.VisionAprilTagsIOPhoton;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.valueholders.DoubleHolder;

public class Vision implements Tunable {
  private final LogFieldsTable fieldsTable = new LogFieldsTable("Vision");
  private final VisionAprilTagsIO[] visionCameras = new VisionAprilTagsIO[CAMERAS.length];

  private static final TrustLevel trustLevelMultiplier = TRUST_LEVEL_MULTIPLIER;
  private static final TrustLevel noOdometryTrustLevelMultiplier = NO_ODOMETRY_TRUST_LEVEL_MULTIPLIER;
  private static final DoubleHolder ambiguityThreshold = new DoubleHolder(AMBIGUITY_THRESHOLD);
  private static final DoubleHolder distanceThreasholdMeters = new DoubleHolder(AVG_DISTANCE_THRESHOLD_METERS);

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
      if (visionData.ambiguity() > ambiguityThreshold.get()) continue;
      if (!FieldConstants.isOnField(visionData.robotPose())) continue;
      double distanceSum = 0;
      for (double distance : visionData.tagsDistancesToCam()) {
        distanceSum += distance;
      }
      double avgDistance = distanceSum / tagsUsed;
      if (avgDistance > distanceThreasholdMeters.get()) continue;
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

  public static class TrustLevel implements Tunable {
    private double xyStdDev;
    private double rotationStdDev;

    public TrustLevel(double xyStdDev, double rotationStdDev) {
      this.xyStdDev = xyStdDev;
      this.rotationStdDev = rotationStdDev;
    }

    public double getXyStdDev() {
        return xyStdDev;
    }

    public double getRotationStdDev() {
        return rotationStdDev;
    }

    public void setXyStdDev(double xyStdDev) {
        this.xyStdDev = xyStdDev;
    }

    public void setRotationStdDev(double rotationStdDev) {
        this.rotationStdDev = rotationStdDev;
    }

    public void multiply(TrustLevel other) {
      this.xyStdDev = other.xyStdDev;
      this.rotationStdDev = other.rotationStdDev;
    }

    @Override
    public void initTunable(TunableBuilder builder) {
      builder.addDoubleProperty("xyStdDev", this::getXyStdDev, this::setXyStdDev);
      builder.addDoubleProperty("rotationStdDev", this::getRotationStdDev, this::setRotationStdDev);
    }
  }

  private static TrustLevel calculateTrustLevel(double stdFactor, int tagsUsed, double avgDistanceToCam, double ambiguity) {
    if (ambiguity == 1 || tagsUsed == 0 || avgDistanceToCam == 0)
      return new TrustLevel(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    double value = Math.pow(avgDistanceToCam, 1.2) / Math.pow(tagsUsed, 2) / Math.pow(1 - ambiguity, 2) * stdFactor;
    TrustLevel result = new TrustLevel(value, value);
    result.multiply(trustLevelMultiplier);
    if (PoseEstimator.getInstance().inCollision() || DriverStation.isDisabled()) {
      result.multiply(noOdometryTrustLevelMultiplier);
    }
    return result;
  }

  @Override
  public void initTunable(TunableBuilder builder) {
    builder.addChild("Trust level multiplier", trustLevelMultiplier);
    builder.addChild("No odoemtry trust level multiplier", trustLevelMultiplier);
    builder.addDoubleProperty("Ambiguity threshold", ambiguityThreshold::get, ambiguityThreshold::set);
    builder.addDoubleProperty("Distance threshold", distanceThreasholdMeters::get, distanceThreasholdMeters::set);
  }
}
