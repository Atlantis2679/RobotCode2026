package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;

import java.io.IOException;

import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.poseestimation.PoseEstimator;

public class VisionConstants {
  public static AprilTagFieldLayout APRTIL_TAGS_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  // static {
  //   try {
  //     APRTIL_TAGS_FIELD_LAYOUT = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2026-rebuilt-welded-edit.json");
  //   } catch (IOException e) {
  //     APRTIL_TAGS_FIELD_LAYOUT = null;
  //   }
  // };

  public static final double TRANSLATION_STD_MULTIPLYER = 0.01;
  public static final double ROTATION_STD_MULTIPLYER = 0.03;

  // public static final double AVG_DISTANCE_DEGREDATION_START_METERS = 0.25;
  public static final double AVG_DISTANCE_THREASHOLD_METERS = 5;
  public static final double AMBIGUITY_THREASHOLD = 0.4;

  public record CameraConfig(String name, double stdFactor, Transform3d robotToCam) {
  };

  public static CameraConfig[] CAMERAS = {
      new CameraConfig(
          "FrontRightCam", 1.0,
          new Transform3d(new Translation3d(0.175, 0.31, 0.39),
              new Rotation3d(Degrees.of(0), Degrees.of(12), Degrees.of(0)))),
      // new CameraConfig(
      //     "FrontLeftCam", 1.0,
      //     new Transform3d(new Translation3d(0.31, 0.19, 0.22),
      //         new Rotation3d(Degrees.of(-1), Degrees.of(-13), Degrees.of(0)))),
      new CameraConfig(
          "BackCam", 1.0,
          new Transform3d(new Translation3d(0.175, -0.21, 0.11),
              new Rotation3d(Degrees.of(0), Degrees.of(-38), Degrees.of(180))))
  };

  public static class Sim {
    public static SimCameraProperties SIM_CAMERA_PROPERTIES = new SimCameraProperties();

    public static VisionSystemSim VISION_SIM = Robot.isSimulation() ? new VisionSystemSim("VisionSim") : null;

    static {
      if (Robot.isSimulation()) {
        VISION_SIM.addAprilTags(APRTIL_TAGS_FIELD_LAYOUT);
        PoseEstimator.registerCallbackOnPoseUpdate(VISION_SIM::update);
        SmartDashboard.putData("VisionSimulation", VISION_SIM.getDebugField());
        configureSimCameraProperties();
      }
    }

    private static void configureSimCameraProperties() {
    }
  }
}
