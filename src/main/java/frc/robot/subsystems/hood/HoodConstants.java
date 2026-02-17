package frc.robot.subsystems.hood;

public final class HoodConstants {
    public static final int MAX_VOLTAGE = 12;
    public static final double CURRENT_LIMIT = 20;
    public static final double HOMING_CURRENT_LIMIT = 1;

    public static final double MOTOR_KV = 917;

    public static final double MAX_ANGLE_DEGREES = 80;
    public static final double MIN_ANGLE_DEGREES = 30;

    public static final double ANGLE_TOLERENCE_DEGREES = 0.1;

    public static final double KP = 1;
    public static final double KI = 5;
    public static final double KD = 0.02;

    public static final double GEAR_RATIO = 360.0/300.0;

    public static final double STUCK_VELOCITY_THRESHOLD_DEG_PER_SEC = 0.01;
    public static final double STUCK_DEBOUNCE_SEC = 0.2; 

    public final class Sim {
        public static final double SIM_KS = 0;
        public static final double SIM_KA = 0;
        public static final double SIM_KV = 0;
        public static final double SIM_KG = 0;

        public static final double JOINT_GEAR_RATIO = 0;
        public static final double JKG_METERS_SQUEARED = 0;
        public static final double ARM_LENGTH_M = 0.0;
    }
}
