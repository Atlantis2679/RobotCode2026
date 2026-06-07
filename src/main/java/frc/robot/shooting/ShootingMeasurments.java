package frc.robot.shooting;

import edu.wpi.first.math.geometry.Transform3d;

public class ShootingMeasurments {
    public static final ShootingState[] ALL_MEASURMENTS_HUB = new ShootingState[] {
        new ShootingState(1.2, 1850, 31),
        new ShootingState(1.45, 1800, 34),
        new ShootingState(1.7, 1900, 36),
        new ShootingState(1.95, 2000, 38),
        new ShootingState(2.2, 2030, 42),
        new ShootingState(2.45, 2100, 42),
        new ShootingState(2.7, 2160, 42),
        new ShootingState(2.95, 2160, 43),
        new ShootingState(3.2, 2180, 45),
        new ShootingState(3.45, 2222, 45),
        new ShootingState(3.7, 2300, 45),
        new ShootingState(3.95, 2380, 47),
        new ShootingState(4.2, 2430, 48),
        new ShootingState(4.45, 2480, 49),
        new ShootingState(4.7, 2530, 50),
        new ShootingState(4.95, 2530, 51),
        new ShootingState(5.2, 2650, 52),
        new ShootingState(5.45, 2670, 51.5),
        // new ShootingState(1.5, 1950, 39),
        // new ShootingState(2, 2050, 42),
        // new ShootingState(2.5, 2100, 44),
        // new ShootingState(3, 2200, 45),
        // new ShootingState(3.5, 2300, 47.5),
        // new ShootingState(4, 2400, 49),
        // new ShootingState(5.5, 2850, 52.5),
    };
    public static final ShootingState[] ALL_MEASURMENTS_DELIVRY = new ShootingState[]{

    };

    public static final Transform3d ROBOT_TO_MEASURMENT_TRANSFORM = new Transform3d();
}