package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.geometry.Transform3d;

public class FlyWheelConstants {
    public static final double MAX_VOLTAGE = 10;
    public static final double STATOR_CURRENT_LIMIT = 90;
    public static final double SUPPLY_CURRENT_LIMIT = 60;
    public static final double SUPPLY_CURRENT_LOWER_LIMIT = 40;
    public static final double SUPPLY_CURRENT_LOWER_TIME = 2;

    public static final double SPEED_TOLERANCE_RPM = 100;

    public static final double KP = 0.0;
    public static final double KI = 0.0;
    public static final double KD = 0.0;

    public static final double KS = 0.21;
    public static final double KA = 0;
    public static final double KV = 0.00186;

    public static final double GEAR_RATIO = 1;

    public static final Transform3d SHOOTER_OUTPUT_TRANSFORM = new Transform3d();

    public static class Sim {
        public static final double SIM_KS = 0;
        public static final double SIM_KA = 0;
        public static final double SIM_KV = 0;
        public static final double SIM_KG = 0;

        public static final double FLYWHEEL_JKgMetersSquared = 0;
    }
}
