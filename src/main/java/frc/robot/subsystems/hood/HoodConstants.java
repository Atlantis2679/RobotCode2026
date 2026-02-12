package frc.robot.subsystems.hood;

public final class HoodConstants {
    public static final int MAX_VOLTAGE = 12;
    public static final int CURRENT_LIMIT = 40;

    public static final double MOTOR_KV = 917;

    public static final double MAX_ANGLE_DEGREES = 0;
    public static final double MIN_ANGLE_DEGREES = 0;
    
    public static final double ANGLE_TOLERENCE_DEGREES = 10;

    public static final double MAX_VELOCITY_DEG_PER_SEC = MAX_VOLTAGE * MOTOR_KV;
    public static final double MAX_ACCELERATION_DEG_PER_SEC_SQUEARD = 10000000;

    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double KS = 0;
    public static final double KA = 0;
    public static final double KV = 1 / MOTOR_KV;

    public static final double GEAR_RATIO = 300/360.0;

    public static final double LIMIT_SWITCH_DEBAUNCER_SEC = 0.06;

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
