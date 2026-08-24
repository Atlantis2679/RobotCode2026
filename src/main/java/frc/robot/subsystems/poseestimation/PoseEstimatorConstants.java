package frc.robot.subsystems.poseestimation;

import frc.robot.subsystems.vision.Vision.TrustLevel;

public final class PoseEstimatorConstants {
    // public static final TrustLevel VISION_Q_STD_DEVS = new double[] {0.000009, 0.000009, 0.000004};
    public static final TrustLevel VISION_Q_STD_DEVS = new TrustLevel(0.000009, 0.000004);
    public static final double ODOMETRY_POSES_BUFFER_SIZE_SEC = 2;
    public static final double MAX_VISION_AGE_SEC = 0.3;
    public static final double IN_COLLISION_DEBOUNCE_SEC = 0.5;
}
