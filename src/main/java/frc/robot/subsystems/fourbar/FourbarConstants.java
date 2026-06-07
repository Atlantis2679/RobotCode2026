package frc.robot.subsystems.fourbar;

public class FourbarConstants {
    public static final double MAX_VOLTAGE = 12;
    public static final int CURRENT_LIMIT = 15;

    public static final double KP = 0.3;
    public static final double KD = 0.0;
    public static final double KI = 0;

    public static final double KG = 0;
    public static final double KS = 0;
    public static final double KV = 0;

    public static final double MAX_VELOCITY_DEG_PER_SEC = 180;
    public static final double MAX_ACCELERATION_DEG_PER_SEC = 1800;

    public static final double STUCK_DEBOUNCE_SEC = 0.1;
    
    public static final double STUCK_VELOCITY_THRESHOLD_DEG_PER_SEC = 0.01;

    public static final double ANGLE_TOLLERANCE = 0.6;

    public static final double ANGLE_OFFSET = 0;

    public static final double MAX_ANGLE_DEGREES = 310;
    public static final double MIN_ANGLE_DEGREES = 250;
}
