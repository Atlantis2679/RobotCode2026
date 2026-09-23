package frc.robot.subsystems.poseestimation;

import frc.robot.subsystems.vision.Vision.TrustLevel;

public final class PoseEstimatorConstants {
    public static final TrustLevel VISION_Q_STD_DEVS = new TrustLevel(0.003,0.002);
    public static final TrustLevel NO_ODOMETRY_TRUST_LEVEL_MULTIPLIER = new TrustLevel(0.2, 0.05);
    public static final double ODOMETRY_POSES_BUFFER_SIZE_SEC = 2;
}
