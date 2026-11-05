package frc.robot.subsystems.poseestimation;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;

public final class PoseEstimatorConstants {
    public static final TrustLevel ODOMETRY_STD_DEVS = new TrustLevel(0.003,0.002);
    public static final Time ODOMETRY_POSES_BUFFER_SIZE = Seconds.of(2);
}
