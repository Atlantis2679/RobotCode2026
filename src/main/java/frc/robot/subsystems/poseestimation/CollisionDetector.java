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

    private void updateAVGs(double[] vals) {
        highCurreentAvarage.update(vals[0]);
        lowCurreentAvarage.update(vals[1]);
    }

    private void resetAVGs() {
        highCurreentAvarage.reset();
        lowCurreentAvarage.reset();
    }

    public void update(CollisionDetectorInfo info) {
        if (info.zAcceleration >= Z_ACCELERATION_THRESHOLD) {
            resetAVGs();
            inCollision = true;
        } if (inCollision || (info.xAcceleration<=STATIC_ACCELERATION_THRESHOLD
                &&info.yAcceleration<=STATIC_ACCELERATION_THRESHOLD)) {
            double[] currents = getHighestX(2, info.currents);
            updateAVGs(currents);
            inCollision = highCurreentAvarage.get() >= HIGH_CURRENT_COLLISION_THRESHOLD
                &&lowCurreentAvarage.get() >= LOW_CURRENT_COLLISION_THRESHOLD;
        } else {
            inCollision = abs(lastXAcceleration-info.xAcceleration)>=JERK_COLLISION_THRESHOLD
                ||abs(lastYAcceleration-info.yAcceleration)>=JERK_COLLISION_THRESHOLD;
            resetAVGs();
        }
        logFieldsTable.recordOutput("X Jerk", abs(lastXAcceleration-info.xAcceleration));
        logFieldsTable.recordOutput("Y Jerk", abs(lastYAcceleration-info.yAcceleration));
        lastXAcceleration = info.xAcceleration();
        lastYAcceleration = info.yAcceleration();
        logFieldsTable.recordOutput("In Collision?", inCollision);
    }

    public boolean inCollision() {
        return inCollision;
    }

    public static record CollisionDetectorInfo(double xAcceleration, double yAcceleration,
            double zAcceleration, double[] currents) {
    }
}
