package frc.robot.subsystems.poseestimation;

public final class PoseEstimatorConstants {
    public static final double[] VISION_Q_STD_DEVS = new double[] {0.000009, 0.000009, 0.000004};
    public static final double ODOMETRY_POSES_BUFFER_SIZE_SEC = 2;
    public static final double IN_COLLISION_DEBOUNCE_SEC = 1;
}
