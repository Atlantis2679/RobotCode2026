package frc.robot.subsystems.poseestimation;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public final class PoseEstimatorConstants {
    public static final TrustLevel ODOMETRY_STD_DEVS = new TrustLevel(0.003,0.002);
    public static final TrustLevel PRE_MATCH_VISION_TRUST_LEVEL_MULTIPLIER = new TrustLevel(0.2, 0.05);
    public static final Time ODOMETRY_POSES_BUFFER_SIZE = Seconds.of(2);
}
