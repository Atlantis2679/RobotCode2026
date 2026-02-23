package frc.robot.subsystems.poseestimation;

import static frc.robot.subsystems.poseestimation.CollisionDetectionConstants.*;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;

import static java.lang.Math.*;

public class CollisionDetector implements Tunable {
    private double lastXAcceleration = 0;
    private double lastYAcceleration = 0;
    private boolean inCollision = false;

    private final LogFieldsTable logFieldsTable;

    private double minCollisionAcceleration = MIN_COLLISION_ACCELERATION;
    private double maxCollisionAcceleration = MAX_COLLISION_ACCELERATION;
    private double jerkCollisionThreshold = JERK_COLLISION_THRESHOLD;
    private double zAccelerationThreshold = Z_ACCELERATION_THRESHOLD;

    public CollisionDetector(LogFieldsTable logFieldsTable) {
        this.logFieldsTable = logFieldsTable;
    }

    private void update(CollisionDetectorInfo info, boolean inCollision) {
        this.lastXAcceleration = info.xAcceleration();
        this.lastYAcceleration = info.yAcceleration();
        this.inCollision = inCollision;
        logFieldsTable.recordOutput("In Collision?", inCollision);
    }

    public void update(CollisionDetectorInfo info) {
        if (info.zAcceleration >= zAccelerationThreshold) {
            update(info, false);
        } else if (!inCollision) {
            if (abs(info.xAcceleration) > STATIC_ACCELERATION_THRESHOLD && abs(info.yAcceleration) > STATIC_ACCELERATION_THRESHOLD) {
                double currentMax1 = 0, currentMax2 = 0;
                for (double current : info.currents) {
                    if (current > currentMax1) {
                        if (current > currentMax2) {
                            currentMax1 = currentMax2;
                            currentMax2 = current;
                        } else {
                            currentMax1 = current;
                        }
                    }
                }
                boolean inCollision = minCollisionAcceleration < abs(currentMax1)
                        && abs(currentMax2) < maxCollisionAcceleration;
                update(info, inCollision);
            } else {
                boolean inCollision = abs(info.xAcceleration - lastXAcceleration) >= jerkCollisionThreshold
                        || abs(info.yAcceleration - lastYAcceleration) >= jerkCollisionThreshold;
                update(info, inCollision);
            }
        } else {
            boolean inCollision = minCollisionAcceleration <= info.xAcceleration
                    && info.xAcceleration <= maxCollisionAcceleration
                    && minCollisionAcceleration <= info.yAcceleration
                    && info.yAcceleration <= maxCollisionAcceleration;
            update(info, inCollision);
        }
    }

    public boolean inCollision() {
        return inCollision;
    }

    public void initTunable(TunableBuilder tunableBuilder) {
        tunableBuilder.addDoubleProperty("Min Collision Acceleration", 
            () -> minCollisionAcceleration,
            (a) -> minCollisionAcceleration = a);
        tunableBuilder.addDoubleProperty("Max Collision Acceleration",
            () -> maxCollisionAcceleration,
            (a) -> maxCollisionAcceleration = a);
        tunableBuilder.addDoubleProperty("Jerk Collision Threshold", 
            () -> jerkCollisionThreshold, 
            (j) -> jerkCollisionThreshold = j);
        tunableBuilder.addDoubleProperty("Z Acceleration Threshold", 
            () -> zAccelerationThreshold, 
            (a) -> zAccelerationThreshold = a);
    }

    public static record CollisionDetectorInfo(double xAcceleration, double yAcceleration,
            double zAcceleration, double[] currents) {
    }
}
