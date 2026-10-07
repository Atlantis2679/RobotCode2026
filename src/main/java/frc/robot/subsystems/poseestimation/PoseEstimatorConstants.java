package frc.robot.subsystems.poseestimation;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public final class PoseEstimatorConstants {
    public static final TrustLevel ODOMETRY_STD_DEVS = new TrustLevel(0.003,0.002);
    public static final TrustLevel PRE_MATCH_VISION_TRUST_LEVEL_MULTIPLIER = new TrustLevel(0.2, 0.05);
    public static final Time ODOMETRY_POSES_BUFFER_SIZE = Seconds.of(2);
    public static final class SkidDetectorConstants {
        // Moudles Skid
        public static final LinearVelocity STATIC_TRANSLATION_VELOCITY_THRESHOLD = MetersPerSecond.of(0.25);
        public static final double MODULE_SKID_THRESHOLD = 0.3;
        public static final LinearVelocity VELOCITY_DISCREPANCY_THRESHOLD = MetersPerSecond.of(0.3);
        public static final Time HOLD_DECAY_SECONDS = Seconds.of(0.4);

        // Imu Slip
        public static final double BIAS_LEARNING_RATE = 0.01; // Use as IMU background learning rate
        public static final Time MAX_VALID_DT = Seconds.of(0.1); // Used to update late arriving updates without interpolation
    }
}
