package frc.robot.allCommands;

import frc.robot.utils.MathUtils.CosineWaveFollower;

public class AllCommandsConstants {
    public static final double FOURBAR_OPEN_VOLTAGE = 2;
    public static final double FOURBAR_CLOSE_VOLTAGE = -3;
    public static final CosineWaveFollower FOURBAR_BOUNCE_WAVE_FOLLOWER = new CosineWaveFollower(-2, 1.5, 0.1);
    public static final double FOURBAR_INTAKE_BOUNCE_MAX_VOLTAGE = 4;
    public static final double SPINDEX_VOLTAGE = 12;
    public static final double INDEXER_VOLTAGE = 12;
    public static final double ROLLER_VOLTAGE = 7;
}
