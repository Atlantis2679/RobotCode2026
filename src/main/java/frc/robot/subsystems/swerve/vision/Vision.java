package frc.robot.subsystems.swerve.vision;

import static frc.robot.subsystems.swerve.vision.VisionConstants.CAMERAS_COUNT;

import frc.robot.subsystems.swerve.vision.io.VisionAprilTagsIO;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Vision {
  private final LogFieldsTable fieldsTable = new LogFieldsTable("Vision");
  private final VisionAprilTagsIO[] visionCameras = new VisionAprilTagsIO[CAMERAS_COUNT];

  public Vision() {
  }
}
