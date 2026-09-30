package frc.robot.subsystems.poseestimation;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;

public final class PoseEstimatorConstants {
    public static final TrustLevel ODOMETRY_STD_DEVS = new TrustLevel(0.003,0.002);
    public static final TrustLevel PRE_MATCH_VISION_TRUST_LEVEL_MULTIPLIER = new TrustLevel(0.2, 0.05);
    public static final double ODOMETRY_POSES_BUFFER_SIZE_SEC = 2;
    public static final class SkidDetectorConstants {
        public static final LinearVelocity STATIC_TRANSLATION_VELOCITY_THRESHOLD = MetersPerSecond.of(0.01);
    }
}
