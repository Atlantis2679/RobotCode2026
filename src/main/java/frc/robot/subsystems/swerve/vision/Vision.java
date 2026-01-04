package frc.robot.subsystems.swerve.vision;

import static frc.robot.subsystems.swerve.vision.VisionConstants.CAMERAS;
import static frc.robot.subsystems.swerve.vision.VisionConstants.HEIGHT_ABOVE_FIELD_THREASHOLD_METERS;

import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Alert;
import frc.robot.subsystems.swerve.PoseEstimator.VisionMesurment;
import frc.robot.subsystems.swerve.vision.VisionConstants.Camera;
import frc.robot.subsystems.swerve.vision.io.VisionAprilTagsIO;
import frc.robot.subsystems.swerve.vision.io.VisionAprilTagsIOPhoton;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlert;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsManager;

public class Vision {
  private final LogFieldsTable fieldsTable = new LogFieldsTable("Vision");
  private final VisionAprilTagsIO[] visionCameras = new VisionAprilTagsIO[CAMERAS.length];

  public Vision() {
    PeriodicAlertsGroup alertsGroup = new PeriodicAlertsGroup("Vision");
    for (int i = 0; i < visionCameras.length; i++) {
      Camera camera = CAMERAS[i];
      visionCameras[i] = new VisionAprilTagsIOPhoton(fieldsTable, camera);
      alertsGroup.addWarningAlert(() -> camera.name(), visionCameras[i].isConnected);
    }
  }

  public List<VisionMesurment> getAllResults(boolean useRotation) {
    List<Pose3d> poses = new ArrayList<>();
    List<Double> ambiguity = new ArrayList<>();
    for (VisionAprilTagsIO io : visionCameras) {
      for (Pose3d pose3d : io.posesEstimates.get()) {
        poses.add(pose3d);
      }
    }
    fieldsTable.recordOutput("Current Poses", poses.toArray(new Pose2d[0]));
    List<VisionMesurment> mesurments = new ArrayList<>();
    for (Pose3d pose : poses) {
      if (usePose(pose, ))
      mesurments.add(new VisionMesurment(pose.toPose2d(), ));
    }
    return mesurments;
  }

  private static boolean isOnField(Pose3d pose) {
    // Check height:
    if (Math.abs(pose.getZ()) > HEIGHT_ABOVE_FIELD_THREASHOLD_METERS) return false;
    return true;
  }

  private static boolean usePose(Pose3d pose, double ambiguity) {

  }

  private static Vector<N3> calculateTrustLevel(Pose3d robotPose, Pose3d mesuredPose, double ambiguity, int posesCount) {
  }
}
