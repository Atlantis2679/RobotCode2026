package frc.robot.subsystems.poseestimation;

import frc.robot.subsystems.vision.Vision.TrustLevel;

public final class PoseEstimatorConstants {
    public static final TrustLevel VISION_Q_STD_DEVS = new TrustLevel(0.003,0.002);
    public static final double ODOMETRY_POSES_BUFFER_SIZE_SEC = 2;
    public static final double MAX_VISION_AGE_SEC = 0.3;
    public static final double IN_COLLISION_DEBOUNCE_SEC = 0.5;
    public static final double ODOMETRY_DRIFT_SIM_TRANSLATION_FACTOR = 1.1;
    public static final double ODOMETRY_DRIFT_SIM_ROTATION_FACTOR = 1.01;
}
