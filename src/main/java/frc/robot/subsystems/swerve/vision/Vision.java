package frc.robot.subsystems.swerve.vision;

import static frc.robot.subsystems.swerve.vision.VisionConstants.CAMERAS_COUNT;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.subsystems.swerve.vision.io.VisionAprilTagsIO;
import frc.robot.subsystems.swerve.vision.io.VisionAprilTagsIOPhoton;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Vision {
  private final LogFieldsTable fieldsTable = new LogFieldsTable("Vision");
  private final VisionAprilTagsIO[] visionCameras = new VisionAprilTagsIO[CAMERAS_COUNT];

  public Vision() {
    for (int i = 0; i < visionCameras.length; i++) {
      visionCameras[i] = new VisionAprilTagsIOPhoton(fieldsTable, i);
    }
  }

  public List<Pose2d> getAllResults() {
    List<Pose2d> poses = new ArrayList<>();
    for (VisionAprilTagsIO io : visionCameras) {
      for (Pose3d pose3d : io.posesEstimates.get()) {
        poses.add(pose3d.toPose2d());
      }
    }
    fieldsTable.recordOutput("Current Poses", poses.toArray(new Pose2d[0]));
    return poses;
  }
}
