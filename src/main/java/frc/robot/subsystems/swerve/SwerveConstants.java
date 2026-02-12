package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class SwerveConstants {
    public static final class Modules {
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

        public static final double TURN_MOTOR_KP = 1.8 * 12;
        public static final double TURN_MOTOR_KI = 0;
        public static final double TURN_MOTOR_KD = 0;

        public static final double MAX_VOLTAGE = 12;
        public static final double MAX_SPEED_MPS = 5;
        // Also used as a refrence for percantage speed calculation - so lowering this MAY cause the modules to faster
        // - always calibrate before changing!

        public static final double PREVENT_JITTERING_MULTIPLAYER = 0.01;

        public static final double DRIVE_GEAR_RATIO = 6.756;
        public static final double TURN_GEAR_RATIO = 12.8;

        public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(2);

        public static final double[] OFFSETS = {
            -26.3671875 + 180,
            -55.72265625 + 180,
            82.177734375 + 180,
            39.7265625 + 180,
        };

        public static final double DRIVE_STATOR_CURRENT_LIMIT = 90;
        public static final double TURN_STATOR_CURRENT_LIMIT = 30;

        public static final double DRIVE_SUPPLY_CURRENT_LIMIT = 70;
        public static final double DRIVE_SUPPLY_CURRENT_LOWER_LIMIT = 40;
        public static final double DRIVE_SUPPLY_CURRENT_LOWER_TIME = 1;
    }

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
            new Translation2d(TRACK_LENGTH_METERS / 2, -TRACK_WIDTH_METERS / 2),
            new Translation2d(-TRACK_LENGTH_METERS / 2, TRACK_WIDTH_METERS / 2),
            new Translation2d(-TRACK_LENGTH_METERS / 2, -TRACK_WIDTH_METERS / 2),
    };

    public static final double GYRO_CONNECTED_DEBUNCER_SECONDS = 0.1;

    public static final class DriverController {
        public static final double DRIVER_MAX_ANGULAR_VELOCITY_RPS = 8;
        public static final double DRIVER_ACCELERATION_LIMIT_MPS = Math.toRadians(720);
        public static final double DRIVER_ANGULAR_ACCELERATION_LIMIT_RPS = 4.5;
        public static final double SENSETIVE_TRANSLATION_MULTIPLIER = 0.3;
        public static final double SENSETIVE_ROTATION_MULTIPLIER = 0.3;
    }

    public static final class CollisionDetectorConstants {
        public static final int CURRENT_AVG_CASES = 1;
        public static final int COLLISION_CURRENT_AVG_CASES = 1;
        public static final double MAJOR_CURRENTS_DIFF = 0;
    }
}
