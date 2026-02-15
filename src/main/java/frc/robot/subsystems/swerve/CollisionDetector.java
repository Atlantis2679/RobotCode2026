package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.CollisionDetectorConstants.*;
import team2679.atlantiskit.logfields.LogFieldsTable;
import static java.lang.Math.*;

public class CollisionDetector {
    private double lastXAcceleration = 0;
    private double lastYAcceleration = 0;
    private boolean inCollision = false;

    private LogFieldsTable logFieldsTable;
    
    public CollisionDetector(LogFieldsTable logFieldsTable) {
        this.logFieldsTable = logFieldsTable;
    }
    
    private void update(CollisionDetectorInfo info, boolean inCollision) {
        this.lastXAcceleration = info.xAcceleration();
        this.lastYAcceleration = info.yAcceleration();
        this.inCollision = inCollision;
        logFieldsTable.recordOutput("In Collision?", inCollision);
    }

    public boolean check(CollisionDetectorInfo info){
        if (info.zAcceleration >= ZERO_ACCELERATION) {
            update(info, true);
            return true;
        } else if (!inCollision) {
            if (abs(info.xAcceleration)>ZERO_ACCELERATION&&abs(info.yAcceleration)>ZERO_ACCELERATION) {
                double currentMax1 = 0, currentMax2 = 0;
                for (double current : info.currents) {
                    if (current>currentMax1) {
                        if (current>currentMax2) {
                            currentMax1 = currentMax2;
                            currentMax2 = current;
                        } else {
                            currentMax1 = current;
                        }
                    }
                }
                boolean toReturn = MIN_COLLISION_ACCELERATION<abs(currentMax1)
                    &&abs(currentMax2)<MAX_COLLISION_ACCELERATION;
                update(info, toReturn);
                return toReturn;
            } else {
                boolean toReturn = abs(info.xAcceleration-lastXAcceleration)>=MAJOR_JERK
                    ||abs(info.yAcceleration-lastYAcceleration)>=MAJOR_JERK;
                update(info, toReturn);
                return toReturn;
            }
        } else {
            boolean toReturn = MIN_COLLISION_ACCELERATION<=info.xAcceleration
                    &&info.xAcceleration<=MAX_COLLISION_ACCELERATION
                &&MIN_COLLISION_ACCELERATION<=info.yAcceleration
                    &&info.yAcceleration<=MAX_COLLISION_ACCELERATION;
            update(info, toReturn);
            return toReturn;
        }
    }
    
    public static record CollisionDetectorInfo(double xAcceleration, double yAcceleration,
        double zAcceleration, double[] currents) {}
}
