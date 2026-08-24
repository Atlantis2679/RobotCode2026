package frc.robot.subsystems.poseestimation;

import static frc.robot.subsystems.poseestimation.CollisionDetectionConstants.*;
import frc.robot.utils.MathUtils.DynamicAverage;
import static frc.robot.utils.MathUtils.getHighestX;

import team2679.atlantiskit.logfields.LogFieldsTable;

import static java.lang.Math.abs;

public class CollisionDetector {
    private double lastXAcceleration = 0;
    private double lastYAcceleration = 0;
    private boolean inCollision = false;

    private DynamicAverage lowCurrentAverage = new DynamicAverage(10);
    private DynamicAverage highCurrentAverage = new DynamicAverage(10);

    private final LogFieldsTable fieldsTable;

    public CollisionDetector(LogFieldsTable fieldsTable) {
        this.fieldsTable = fieldsTable;
    }

    private void updateAVGs(double[] vals) {
        highCurrentAverage.update(vals[0]);
        lowCurrentAverage.update(vals[1]);
    }

    private void resetAVGs() {
        highCurrentAverage.reset();
        lowCurrentAverage.reset();
    }

    public void update(CollisionDetectorInfo info) {
        if (info.zAcceleration >= Z_ACCELERATION_THRESHOLD) {
            resetAVGs();
            inCollision = true;
        } if (inCollision || (info.xAcceleration<=STATIC_ACCELERATION_THRESHOLD
                &&info.yAcceleration<=STATIC_ACCELERATION_THRESHOLD)) {
            double[] currents = getHighestX(2, info.currents);
            updateAVGs(currents);
            inCollision = highCurrentAverage.get() >= HIGH_CURRENT_COLLISION_THRESHOLD
                &&lowCurrentAverage.get() >= LOW_CURRENT_COLLISION_THRESHOLD;
        } else {
            inCollision = abs(lastXAcceleration-info.xAcceleration)>=JERK_COLLISION_THRESHOLD
                ||abs(lastYAcceleration-info.yAcceleration)>=JERK_COLLISION_THRESHOLD;
            resetAVGs();
        }
        fieldsTable.recordOutput("X Jerk", abs(lastXAcceleration-info.xAcceleration));
        fieldsTable.recordOutput("Y Jerk", abs(lastYAcceleration-info.yAcceleration));
        lastXAcceleration = info.xAcceleration();
        lastYAcceleration = info.yAcceleration();
        fieldsTable.recordOutput("In Collision?", inCollision);
    }

    public boolean inCollision() {
        return inCollision;
    }

    public static record CollisionDetectorInfo(double xAcceleration, double yAcceleration,
            double zAcceleration, double[] currents) {
    }
}
