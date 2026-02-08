package frc.robot.subsystems.flywheel;

public class FlyWheelConstants {
    public static final double MAX_VOLTAGE = 10;
    public static final double STATOR_CURRENT_LIMIT = 90;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 2;

    public static final double SPEED_TOLERANCE_RPM = 50;

    public static final double KP = 7;
    public static final double KI = 0.02;
    public static final double KD = 0.01;

    public static final double KS = 0.01;
    public static final double KA = 1;
    public static final double KV = 1;

    public static final double GEAR_RATIO = 1;

    public static class Sim {
        public static final double SIM_KS = 0;
        public static final double SIM_KA = 0;
        public static final double SIM_KV = 0;
        public static final double SIM_KG = 0;

        public static final double FLYWHEEL_JKgMetersSquared = 0;
    }
}
