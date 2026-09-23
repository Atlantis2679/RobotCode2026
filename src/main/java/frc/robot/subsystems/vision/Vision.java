package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.AMBIGUITY_THRESHOLD;
import static frc.robot.subsystems.vision.VisionConstants.AVG_DISTANCE_THRESHOLD_METERS;
import static frc.robot.subsystems.vision.VisionConstants.CAMERAS;
import static frc.robot.subsystems.vision.VisionConstants.TRUST_LEVEL_MULTIPLIER;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.valueholders.DoubleHolder;

public class Vision extends SubsystemBase implements Tunable {
  private final LogFieldsTable fieldsTable = new LogFieldsTable("Vision");
  private final VisionAprilTagsIO[] visionCameras = new VisionAprilTagsIO[CAMERAS.length];

  private static final TrustLevel trustLevelMultiplier = TRUST_LEVEL_MULTIPLIER;
  private static final DoubleHolder ambiguityThreshold = new DoubleHolder(AMBIGUITY_THRESHOLD);
  private static final DoubleHolder distanceThresholdMeters = new DoubleHolder(AVG_DISTANCE_THRESHOLD_METERS);

  public Vision() {
    PeriodicAlertsGroup alertsGroup = new PeriodicAlertsGroup("Vision");
    for (int i = 0; i < visionCameras.length; i++) {
      CameraConfig camera = CAMERAS[i];
      visionCameras[i] = new VisionAprilTagsIOPhoton(fieldsTable, camera);
      VisionAprilTagsIO io = visionCameras[i];
      alertsGroup.addWarningAlert(() -> camera.name() + " Disconnected!", () -> !io.isConnected.getAsBoolean());
    }
    TunablesManager.add(getName(), (Tunable) this);
  }

  private static List<VisionMeasurement> getAllResultsInIO(VisionAprilTagsIO io) {
    VisionData[] visionDataArr = io.visionData.get();
    List<VisionMeasurement> visionMeasurements = new ArrayList<>();
    double stdFactor = io.getCameraConfig().stdFactor();
    for (VisionData visionData : visionDataArr) {
      if (visionData.ambiguity() > ambiguityThreshold.get()) continue;
      if (visionData.avgDistanceToCam() > distanceThresholdMeters.get()) continue;
      if (!FieldConstants.isOnField(visionData.robotPose())) continue;
      TrustLevel trustLevels = calculateTrustLevel(stdFactor, visionData.tagsUsed(), visionData.avgDistanceToCam(), visionData.ambiguity());
      visionMeasurements.add(new VisionMeasurement(visionData.robotPose().toPose2d(), trustLevels, visionData.timestamp()));
    }
    return visionMeasurements;
  }

  private List<VisionMeasurement> getAllResults() {
    List<VisionMeasurement> measurements = new ArrayList<>();
    for (VisionAprilTagsIO io : visionCameras) {
      for (VisionMeasurement measurment : getAllResultsInIO(io)) {
        measurements.add(measurment);
      }
    }
    fieldsTable.recordOutput("Vision measurements", measurements.toArray(new VisionMeasurement[0]));
    return measurements;
  }

  @Override
  public void periodic() {
    for (VisionMeasurement mesurement : getAllResults()) {
      PoseEstimator.getInstance().addVisionMeasurement(mesurement);
    }
  }

  @Override
  public void simulationPeriodic() {
    VisionConstants.Sim.VISION_SIM.update(PoseEstimator.getInstance().getOdometryPose());
  }

  public static class TrustLevel implements StructSerializable, Tunable {
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
      this.xyStdDev = this.xyStdDev * other.xyStdDev;
      this.rotationStdDev = this.rotationStdDev * other.rotationStdDev;
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addDoubleProperty("xyStdDev", this::getXyStdDev, this::setXyStdDev);
        builder.addDoubleProperty("rotationStdDev", this::getRotationStdDev, this::setRotationStdDev);
    }

    public static final Struct<TrustLevel> struct = new Struct<>() {
      public Class<TrustLevel> getTypeClass() { return TrustLevel.class; }
      public String getTypeName() { return "TrustLevel"; }
      public int getSize() { return kSizeDouble * 2; }
      public String getSchema() { return "double xyStdDev;double rotationStdDev"; }
      public TrustLevel unpack(ByteBuffer bb) {
        return new TrustLevel(bb.getDouble(), bb.getDouble());
      }
      public void pack(ByteBuffer bb, TrustLevel v) {
        bb.putDouble(v.getXyStdDev());
        bb.putDouble(v.getRotationStdDev());
      }
    };
  }

  private static TrustLevel calculateTrustLevel(double stdFactor, int tagsUsed, double avgDistanceToCam, double ambiguity) {
    if (ambiguity == 1 || tagsUsed == 0 || avgDistanceToCam == 0)
      return new TrustLevel(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
    double value = Math.pow(avgDistanceToCam, 1.2) / Math.pow(tagsUsed, 2) / Math.pow(1 - ambiguity, 2) * stdFactor;
    TrustLevel result = new TrustLevel(value, value);
    result.multiply(trustLevelMultiplier);
    return result;
  }

  @Override
  public void initTunable(TunableBuilder builder) {
    builder.addChild("Trust level multiplier", trustLevelMultiplier);
    builder.addDoubleProperty("Ambiguity threshold", ambiguityThreshold::get, ambiguityThreshold::set);
    builder.addDoubleProperty("Distance threshold", distanceThresholdMeters::get, distanceThresholdMeters::set);
  }
}
