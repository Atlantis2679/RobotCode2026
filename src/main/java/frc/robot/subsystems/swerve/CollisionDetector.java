package frc.robot.subsystems.swerve;

import frc.robot.utils.DynamicAvarage;
import static frc.robot.subsystems.swerve.SwerveConstants.CollisionDetectorConstants.*;

public class CollisionDetector {
    private DynamicAvarage accelerationAvg = new DynamicAvarage(ACCELERATION_SECONDS),
        currentFR = new DynamicAvarage(CURRENT_SECONDS),
        currentBR = new DynamicAvarage(CURRENT_SECONDS), 
        currentBL = new DynamicAvarage(CURRENT_SECONDS),
        currentFL = new DynamicAvarage(CURRENT_SECONDS);

    public CollisionDetector() {}

    public void update (double acceleration, double[] currents) {
        accelerationAvg.update(acceleration);
        currentFR.update(currents[0]);
        currentBR.update(currents[1]);
        currentBL.update(currents[2]);
        currentFL.update(currents[3]);
    }

    public boolean isColliding(double acceleration, double[] currents) {
        return true;




    } 
}
