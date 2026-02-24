package frc.robot.subsystems.poseestimation;

import static frc.robot.subsystems.poseestimation.CollisionDetectionConstants.*;
import frc.robot.utils.MathUtils.DynamicAvarage;
import static frc.robot.utils.MathUtils.getHighestX;

import team2679.atlantiskit.logfields.LogFieldsTable;

import static java.lang.Math.abs;

public class CollisionDetector {
    private double lastXAcceleration = 0;
    private double lastYAcceleration = 0;
    private boolean inCollision = false;

    private DynamicAvarage lowCurreentAvarage = new DynamicAvarage(10);
    private DynamicAvarage highCurreentAvarage = new DynamicAvarage(10);

    private final LogFieldsTable logFieldsTable;

    public CollisionDetector(LogFieldsTable logFieldsTable) {
        this.logFieldsTable = logFieldsTable;
    }

    private boolean updateWrapper(CollisionDetectorInfo info, boolean inCollision) {
        logFieldsTable.recordOutput("X Jerk", abs(lastXAcceleration-info.xAcceleration));
        logFieldsTable.recordOutput("Y Jerk", abs(lastYAcceleration-info.yAcceleration));
        lastXAcceleration = info.xAcceleration();
        lastYAcceleration = info.yAcceleration();
        this.inCollision = inCollision;
        logFieldsTable.recordOutput("In Collision?", inCollision);
        return inCollision;
    }

    private void updateAVGs(double[] vals) {
        highCurreentAvarage.update(vals[0]);
        lowCurreentAvarage.update(vals[1]);
    }

    private void updateAVGs() {
        highCurreentAvarage.reset();
        lowCurreentAvarage.reset();
    }

    public boolean check(CollisionDetectorInfo info) {
        if (info.zAcceleration >= Z_ACCELERATION_THRESHOLD) {
            updateWrapper(info, false);
            updateAVGs();
            return true;
        } if (inCollision || (info.xAcceleration<=STATIC_ACCELERATION_THRESHOLD
                &&info.yAcceleration<=STATIC_ACCELERATION_THRESHOLD)) {
            double[] currents = getHighestX(2, info.currents);
            updateAVGs(currents);
            return updateWrapper(info, highCurreentAvarage.get()>=HIGH_CURRENT_COLLISION_THRESHOLD
                &&lowCurreentAvarage.get()>=LOW_CURRENT_COLLISION_THRESHOLD);
        } else {
            boolean toReturn = abs(lastXAcceleration-info.xAcceleration)>=JERK_COLLISION_THRESHOLD
                ||abs(lastYAcceleration-info.yAcceleration)>=JERK_COLLISION_THRESHOLD;
            updateAVGs();
            return updateWrapper(info, toReturn);
        }
    }

    public static record CollisionDetectorInfo(double xAcceleration, double yAcceleration,
            double zAcceleration, double[] currents) {
    }
}
