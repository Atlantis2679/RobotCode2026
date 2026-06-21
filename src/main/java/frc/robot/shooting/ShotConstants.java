package frc.robot.shooting;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.shooting.ProjectileSimulator.SimParameters;
import frc.robot.subsystems.hood.HoodConstants;

public final class ShotConstants {
    public final static SimParameters PARAMETERS = new SimParameters(
        0.215,
        0.1501,
        0.47,
        0.2,
        1.225,
        0.43,
        0.1016,
        1.83,
        0.6,
        45.0,
        0.001,
        1500, 5500, 25, 5.0
    );

    public static final ShotCalculator.Config CONFIG = new ShotCalculator.Config();
    static {
        CONFIG.launcherOffsetX = 0.293653;
        CONFIG.launcherOffsetY = 0.009185;
        CONFIG.headingSpeedScalar = 1.0;
    }

    public static final double MAX_ANGLE_DEG = HoodConstants.MAX_ANGLE_DEGREES;
    public static final double MIN_ANGLE_DEG = HoodConstants.MIN_ANGLE_DEGREES;
    public static final double ANGLE_STEP = 1;

    public static final double POSE_CONFIDENCE = 0.9;

    // public static final Translation2d RED_HUB = new Translation2d(4.6, 4.0);
    public static final Translation2d RED_HUB = new Translation2d(0.0, 0.0);
    public static final Translation2d BLUE_HUB = new Translation2d(4.6, 4.0);
    public static final Translation2d RED_HUB_HEADING = new Translation2d(1, 0);
    public static final Translation2d BLUE_HUB_HEADING = new Translation2d(0, 0);

    public static final double SHOT_CONFIDENCE_FILTER_THRESHOLD = 0.6;

    public static final ShotLUT SHOT_MEASURMENTS = new ShotLUT();

    static {
        SHOT_MEASURMENTS.put(0.0, 0.0, 0.0, 0.0);
    }
}
