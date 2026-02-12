package frc.robot.subsystems.swerve;

import frc.robot.utils.DynamicAvarage;
import team2679.atlantiskit.logfields.LogFieldsTable;

import static frc.robot.subsystems.swerve.SwerveConstants.CollisionDetectorConstants.*;

public class CollisionDetector {
    private double lastXAcceleration = 0;
    private double lastYAcceleration = 0;
    private boolean inCollision = false;

    private DynamicAvarage[] currentsAvg = {
        new DynamicAvarage(CURRENT_AVG_CASES), new DynamicAvarage(CURRENT_AVG_CASES),
        new DynamicAvarage(CURRENT_AVG_CASES), new DynamicAvarage(CURRENT_AVG_CASES)};

    private LogFieldsTable logFieldsTable;
    
    public CollisionDetector(LogFieldsTable logFieldsTable) {
        this.logFieldsTable = logFieldsTable;
    }

    public boolean check(double xAcceleration, double yAcceleration){
        if (!inCollision) {
            boolean toReturn = Math.abs(xAcceleration-lastXAcceleration)>=MAJOR_JERK
                ||Math.abs(yAcceleration-lastYAcceleration)>=MAJOR_JERK;
            lastXAcceleration = xAcceleration;
            lastYAcceleration = yAcceleration;
            logFieldsTable.recordOutput("Collision", toReturn);
            return toReturn;
        } else {
            inCollision = Math.abs(xAcceleration)<=COLLISION_ACCELERATION
                &&Math.abs(yAcceleration)<=COLLISION_ACCELERATION;
            logFieldsTable.recordOutput("Collision", inCollision);
            return inCollision;
        }
    }
    
}
