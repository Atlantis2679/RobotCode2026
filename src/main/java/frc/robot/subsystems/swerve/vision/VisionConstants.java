package frc.robot.subsystems.swerve.vision;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
  public static final AprilTagFieldLayout APRTIL_TAGS_FIELD_LAYOUT = AprilTagFieldLayout
      .loadField(AprilTagFields.k2025ReefscapeWelded);

  public static final double TRANSLATION_STD_MULTIPLYER = 0.01;
  public static final double ROTATION_STD_MULTIPLYER = 0.01;

  // isOnField:
  public static final double HEIGHT_ABOVE_FIELD_THREASHOLD_METERS = 0.1;
  // public static final double 

  public record Camera(String name, double stdFactor, Transform3d robotToCam) {
  };

  public static Camera[] CAMERAS = {
      new Camera(
          "FrontRightCam", 1.0,
          new Transform3d(new Translation3d(0.285, -0.19, 0.355),
              new Rotation3d(Degrees.of(-0.76), Degrees.of(8.5), Degrees.of(-4)))),
      new Camera(
          "FrontLeftCam", 1.0,
          new Transform3d(new Translation3d(0.31, 0.19, 0.22),
              new Rotation3d(Degrees.of(-1), Degrees.of(-13), Degrees.of(0)))),
      new Camera(
          "BackCam", 1.0,
          new Transform3d(new Translation3d(-0.27, 0, 0.54),
              new Rotation3d(Degrees.of(0), Degrees.of(-23.5), Degrees.of(154))))
  };
}