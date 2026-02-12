package frc.robot.subsystems.swerve;

import frc.robot.utils.DynamicAvarage;
import static frc.robot.subsystems.swerve.SwerveConstants.CollisionDetectorConstants.*;

public class CollisionDetector {
    private double last_acceleration = 0;
    private boolean inCollision = false;

    private DynamicAvarage[] currentsAvg = {
        new DynamicAvarage(CURRENT_AVG_CASES), new DynamicAvarage(CURRENT_AVG_CASES),
        new DynamicAvarage(CURRENT_AVG_CASES), new DynamicAvarage(CURRENT_AVG_CASES)};
    private DynamicAvarage[] collisionCurrentsAvg = {
        new DynamicAvarage(COLLISION_CURRENT_AVG_CASES), new DynamicAvarage(COLLISION_CURRENT_AVG_CASES),
        new DynamicAvarage(COLLISION_CURRENT_AVG_CASES), new DynamicAvarage(COLLISION_CURRENT_AVG_CASES)};
    
    public CollisionDetector() {}

    public boolean check(double acceleration, double[] currents){
        if (inCollision) {
            for (int i = 0; i<4; ++i) {
                if (currents[i])
            }
        }
    }
    
}
