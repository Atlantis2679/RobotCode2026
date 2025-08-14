package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;

public final class SwerveConstants {
    public static final String getModuleName(int moduleNum) {
        switch (moduleNum) {
            case 0:
                return "FL";
            case 1:
                return "FR"; 
            case 2:
                return "BL";
            case 3:
                return "BR";
            default:
                return "None";
        }
    }

    public static final double TURN_MOTOR_KP = 0;
    public static final double TURN_MOTOR_KI = 0;
    public static final double TURN_MOTOR_KD = 0;

    public static final double MAX_VOLTAGE = 12.5;
    public static final double MAX_SPEED_MPS = 0;

    public static final double DRIVE_GEAR_RATIO = 0;
    public static final double TURN_GEAR_RATIO = 0;

    public final class Sim {        
        public static final double DRIVE_MOTOR_MOMENT_OF_INERTIA = 0.025;
        public static final double TURN_MOTOR_MOMENT_OF_INERTIA = 0.004;

        public static final double SIM_TURN_MOTOR_KP = 0;
        public static final double SIM_TURN_MOTOR_KI = 0;
        public static final double SIM_TURN_MOTOR_KD = 0;    
    }

    public static final double TRACK_LENGTH_METERS = 0.595;
    public static final double TRACK_WIDTH_METERS = 0.595;

    public static final Translation2d[] MODULES_LOCATIONS = {
        new Translation2d(TRACK_LENGTH_METERS / 2, TRACK_WIDTH_METERS / 2),
        new Translation2d(TRACK_LENGTH_METERS / 2, TRACK_WIDTH_METERS / 2),
        new Translation2d(TRACK_LENGTH_METERS / 2, TRACK_WIDTH_METERS / 2),
        new Translation2d(TRACK_LENGTH_METERS / 2, TRACK_WIDTH_METERS / 2),
    };
}
