package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Degrees;

import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.poseestimation.PoseEstimator;
import frc.robot.subsystems.vision.Vision.TrustLevel;

public class VisionConstants {
  public static final AprilTagFieldLayout APRTIL_TAGS_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  // static {
  //   try {
  //     APRTIL_TAGS_FIELD_LAYOUT = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2026-rebuilt-welded-edit.json");
  //   } catch (IOException e) {
  //     APRTIL_TAGS_FIELD_LAYOUT = null;
  //   }
  // };

  public static final TrustLevel TRUST_LEVEL_MULTIPLIER = new TrustLevel(0.01, 0.03);

  public static final double AVG_DISTANCE_THRESHOLD_METERS = 3;
  public static final double AMBIGUITY_THRESHOLD = 0.4;

  public static final double NO_ODOMETRY_STD_MULTIPLIER = 0.2;

  public record CameraConfig(String name, double stdFactor, Transform3d robotToCam) {
  };

  public static CameraConfig[] CAMERAS = {
    new CameraConfig(
      "LeftFront", 1.0,
      new Transform3d(new Translation3d(0.297, 0.185, 0.177),
        new Rotation3d(Degrees.of(0), Degrees.of(-48), Degrees.of(22.5)))),
      new CameraConfig(
          "RightFront", 1.0,
          new Transform3d(new Translation3d(0.297, 0.155, 0.177),
              new Rotation3d(Degrees.of(0), Degrees.of(-48), Degrees.of(-22.5)))),
      new CameraConfig(
          "BackCam", 1.0,
          new Transform3d(new Translation3d(0.17, -0.325, 0.505),
              new Rotation3d(Degrees.of(.0), Degrees.of(0), Degrees.of(180))))
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
