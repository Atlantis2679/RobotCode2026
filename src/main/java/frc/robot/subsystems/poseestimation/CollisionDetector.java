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

    private double zeroAcceleration = ZERO_ACCELERATION;
    private double minCollisionAcceleration = MIN_COLLISION_ACCELERATION;
    private double maxCollisionAcceleration = MAX_COLLISION_ACCELERATION;
    private double majorJerk = MAJOR_JERK;

    public CollisionDetector(LogFieldsTable logFieldsTable) {
        this.logFieldsTable = logFieldsTable;
    }

    private void update(CollisionDetectorInfo info, boolean inCollision) {
        this.lastXAcceleration = info.xAcceleration();
        this.lastYAcceleration = info.yAcceleration();
        this.inCollision = inCollision;
        logFieldsTable.recordOutput("In Collision?", inCollision);
    }

    public boolean check(CollisionDetectorInfo info) {
        if (info.zAcceleration >= zeroAcceleration) {
            update(info, false);
            return true;
        } else if (!inCollision) {
            if (abs(info.xAcceleration) > zeroAcceleration && abs(info.yAcceleration) > zeroAcceleration) {
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
                boolean toReturn = minCollisionAcceleration < abs(currentMax1)
                        && abs(currentMax2) < maxCollisionAcceleration;
                update(info, toReturn);
                return toReturn;
            } else {
                boolean toReturn = abs(info.xAcceleration - lastXAcceleration) >= majorJerk
                        || abs(info.yAcceleration - lastYAcceleration) >= majorJerk;
                update(info, toReturn);
                return toReturn;
            }
        } else {
            boolean toReturn = minCollisionAcceleration <= info.xAcceleration
                    && info.xAcceleration <= maxCollisionAcceleration
                    && minCollisionAcceleration <= info.yAcceleration
                    && info.yAcceleration <= maxCollisionAcceleration;
            update(info, toReturn);
            return toReturn;
        }
    }

    public void initTunable(TunableBuilder tunableBuilder) {
        tunableBuilder.addDoubleProperty("Zero Acceleration", () -> zeroAcceleration, (a) -> zeroAcceleration = a);
        tunableBuilder.addDoubleProperty("Min Collision Acceleration", () -> minCollisionAcceleration,
                (a) -> minCollisionAcceleration = a);
        tunableBuilder.addDoubleProperty("Max Collision Acceleration", () -> maxCollisionAcceleration,
                (a) -> maxCollisionAcceleration = a);
        tunableBuilder.addDoubleProperty("Major Jerk", () -> majorJerk, (j) -> majorJerk = j);
    }

    public static record CollisionDetectorInfo(double xAcceleration, double yAcceleration,
            double zAcceleration, double[] currents) {
    }
}
