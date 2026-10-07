package frc.robot.subsystems.poseestimation;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.poseestimation.PoseEstimatorConstants.SkidDetectorConstants.*;
import static frc.robot.utils.MathUtils.median;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.robot.subsystems.swerve.SwerveConstants;
import team2679.atlantiskit.logfields.LogFieldsTable;

import java.util.Arrays;

public class SkidDetector {
    private final LogFieldsTable fieldsTable;
    private final Translation2d[] moduleLocations = SwerveConstants.MODULES_LOCATIONS;
    private final ImuSlipLayer imuSlipLayer = new ImuSlipLayer();

    private double heldSeverity = 0.0;
    private Time lastTimestamp = Seconds.of(0);

    public SkidDetector(LogFieldsTable fieldsTable) {
        this.fieldsTable = fieldsTable;
    }

    public double update(
            Time timestamp,
            SwerveModuleState[] moduleStates,
            Rotation2d yaw,
            AngularVelocity yawAngluarVelocity,
            Translation2d robotIMUAcceleration,
            boolean robotDisabled) {
        Translation2d[] translationalVelocities =
                computeTranslationalVelocities(moduleStates, yawAngluarVelocity);
        Translation2d medianTranslationalVelocityRobot = medianTranslation(translationalVelocities);
        double[] moduleSkids =
                computeModuleSkids(moduleStates, translationalVelocities, medianTranslationalVelocityRobot);
        double maxModuleSkid = Arrays.stream(moduleSkids).max().getAsDouble();

        double velocityDiscrepancy = imuSlipLayer.update(
                timestamp, robotIMUAcceleration, medianTranslationalVelocityRobot, yaw, robotDisabled);

        double moduleSkidSeverity = maxModuleSkid / MODULE_SKID_THRESHOLD;
        double imuSkidSeverity = velocityDiscrepancy / VELOCITY_DISCREPANCY_THRESHOLD.in(MetersPerSecond);
        double combinedSeverity = Math.max(moduleSkidSeverity, imuSkidSeverity);
        heldSeverity = holdWithDecay(timestamp, combinedSeverity);

        fieldsTable.recordOutput("ModuleSkids", moduleSkids);
        fieldsTable.recordOutput("MedianTranslationalVelocity", medianTranslationalVelocityRobot);
        fieldsTable.recordOutput("ImuVelocityDiscrepancy", velocityDiscrepancy);
        fieldsTable.recordOutput("ModuleSkidSeverity", moduleSkidSeverity);
        fieldsTable.recordOutput("ImuSkidSeverity", imuSkidSeverity);
        fieldsTable.recordOutput("HeldSeverity", heldSeverity);

        return heldSeverity;
    }

    private Translation2d[] computeTranslationalVelocities(
            SwerveModuleState[] moduleStates, AngularVelocity yawAgularVelocity) {
        Translation2d[] translationalVelocities = new Translation2d[moduleStates.length];
        for (int i = 0; i < moduleStates.length; i++) {
            Translation2d measuredVelocity =
                    new Translation2d(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle);
            Translation2d rotationalVelocity = new Translation2d(
                    -yawAgularVelocity.in(RadiansPerSecond) * moduleLocations[i].getY(),
                    yawAgularVelocity.in(RadiansPerSecond) * moduleLocations[i].getX());
            translationalVelocities[i] = measuredVelocity.minus(rotationalVelocity);
        }
        return translationalVelocities;
    }

    private static double[] computeModuleSkids(
            SwerveModuleState[] moduleStates,
            Translation2d[] translationalVelocities,
            Translation2d medianTranslationalVelocity) {
        double[] moduleSkids = new double[moduleStates.length];
        double staticThreshold = STATIC_TRANSLATION_VELOCITY_THRESHOLD.in(MetersPerSecond);

        boolean anyModuleMoving = Arrays.stream(moduleStates)
                .anyMatch(state -> Math.abs(state.speedMetersPerSecond) >= staticThreshold);
        if (!anyModuleMoving) {
            return moduleSkids;
        }

        double denominator = Math.max(medianTranslationalVelocity.getNorm(), staticThreshold);
        for (int i = 0; i < moduleStates.length; i++) {
            moduleSkids[i] = translationalVelocities[i].minus(medianTranslationalVelocity).getNorm() / denominator;
        }
        return moduleSkids;
    }

    private double holdWithDecay(Time timestamp, double combinedSeverity) {
        Time dt = timestamp.minus(lastTimestamp);
        lastTimestamp = timestamp;
        double decayedSeverity = heldSeverity * Math.exp(-dt.div(HOLD_DECAY_SECONDS).magnitude());
        return Math.max(combinedSeverity, decayedSeverity);
    }

    private static Translation2d medianTranslation(Translation2d[] translations) {
        double xMedian = median(Arrays.stream(translations).mapToDouble(Translation2d::getX).toArray());
        double yMedian = median(Arrays.stream(translations).mapToDouble(Translation2d::getY).toArray());
        return new Translation2d(xMedian, yMedian);
    }

    private static class ImuSlipLayer {
        private static final Time LEAK_TIME_CONSTANT_SECONDS = Seconds.of(0.1); // The time i
        private static final double BIAS_LEARNING_RATE = 0.01; // Use as IMU background learning rate
        private static final Time MAX_VALID_DT = Seconds.of(0.1); // Used to update late arriving updates without interpolation

        private Translation2d imuVelocityEstimateField = new Translation2d();
        private Translation2d accelerometerBiasRobot = new Translation2d(); // Used to remove IMU background noise
        private Time lastTimestamp = Seconds.of(0);

        double update(
                Time timestamp,
                Translation2d imuAccelerationRobot,
                Translation2d medianTranslationalVelocityRobot,
                Rotation2d gyroYaw,
                boolean robotDisabled) {
            Translation2d odometryVelocityField = medianTranslationalVelocityRobot.rotateBy(gyroYaw);

            Time dt = timestamp.minus(lastTimestamp);
            lastTimestamp = timestamp;
            if (dt.baseUnitMagnitude() == 0 || dt.gt(MAX_VALID_DT)) {
                imuVelocityEstimateField = odometryVelocityField;
                return 0.0;
            }

            if (robotDisabled) {
                accelerometerBiasRobot = accelerometerBiasRobot.interpolate(imuAccelerationRobot, BIAS_LEARNING_RATE);
            }

            Translation2d imuAccelerationField = imuAccelerationRobot.minus(accelerometerBiasRobot).rotateBy(gyroYaw);

            Translation2d pullTowardOdometry = odometryVelocityField
                    .minus(imuVelocityEstimateField)
                    .times(dt.div(LEAK_TIME_CONSTANT_SECONDS).magnitude());
            imuVelocityEstimateField = imuVelocityEstimateField
                    .plus(imuAccelerationField.times(dt.in(Seconds)))
                    .plus(pullTowardOdometry);

            return odometryVelocityField.minus(imuVelocityEstimateField).getNorm();
        }
    }
}

