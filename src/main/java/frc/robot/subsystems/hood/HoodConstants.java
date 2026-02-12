package frc.robot.subsystems.hood;

public final class HoodConstants {
    public static final int MAX_VOLTAGE = 12;
    public static final int CURRENT_LIMIT = 5;

    public static final double MOTOR_KV = 917;

    public static final double MAX_ANGLE_DEGREES = 40;
    public static final double MIN_ANGLE_DEGREES = 10;

    public static final double ANGLE_OFFSET = 10;

    public static final double ANGLE_TOLERENCE_DEGREES = 0.1;

    public static final double KP = 0.85;
    public static final double KI = 35;
    public static final double KD = 0.03;

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
