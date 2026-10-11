package frc.robot.subsystems.poseestimation;

import static frc.robot.subsystems.poseestimation.PoseEstimatorConstants.PRE_MATCH_VISION_TRUST_LEVEL_MULTIPLIER;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.poseestimation.PoseEstimatorConstants.ODOMETRY_POSES_BUFFER_SIZE;
import static frc.robot.subsystems.poseestimation.PoseEstimatorConstants.ODOMETRY_STD_DEVS;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;

public class PoseEstimator implements Tunable {
    private static final List<Consumer<Pose2d>> callbackOnPoseUpdate = new ArrayList<>();
    private static final PoseEstimator instance = new PoseEstimator();

    private Pose2d odometryPose = Pose2d.kZero;
    private Pose2d estimatedPose = Pose2d.kZero;

    private final TimeInterpolatableBuffer<Pose2d> odometryPosesBuffer =
        TimeInterpolatableBuffer.createBuffer(ODOMETRY_POSES_BUFFER_SIZE.in(Seconds));
    
    private final LogFieldsTable fieldsTable = new LogFieldsTable("PoseEstimator");
    
    private TunableTrustLevel odometryTrustLevel = new TunableTrustLevel(ODOMETRY_STD_DEVS);
    private TunableTrustLevel preMatchVisionTrustLevelMultiplier = new TunableTrustLevel(PRE_MATCH_VISION_TRUST_LEVEL_MULTIPLIER);

    private RotationalSensorHelper yawRotationalSensorHelper = new RotationalSensorHelper(0);
    private Optional<Rotation2d> lastGyroAngle = Optional.empty();

    private Time lastResetTimestamp = Seconds.of(0);

    private PoseEstimator() {}

    public static PoseEstimator getInstance() {
        return instance;
    }

    public void addOdometryMeasurement(OdometryMeasurement measurement) {
        Twist2d twist2d = measurement.kinematics.toTwist2d(measurement.modulePositionsDelta);
        if (measurement.gyroAngle.isPresent()) {
          if (lastGyroAngle.isEmpty()) {
            yawRotationalSensorHelper.setOffset(measurement.gyroAngle.get().minus(odometryPose.getRotation()).getDegrees());
          } else {
            twist2d.dtheta = measurement.gyroAngle.get().minus(lastGyroAngle.get()).getRadians();
          }
          lastGyroAngle = Optional.of(measurement.gyroAngle.get());
        } else {
          lastGyroAngle = Optional.empty();
        }
        fieldsTable.recordOutput("OdometryTwist", twist2d);
        Pose2d lastOdometryPose = odometryPose;
        odometryPose = odometryPose.exp(twist2d);
        yawRotationalSensorHelper.update(odometryPose.getRotation().getDegrees());
        fieldsTable.recordOutput("Current Odometry Pose", odometryPose);
        fieldsTable.recordOutput("YawAngluarVelocity", yawRotationalSensorHelper.getVelocity());
        odometryPosesBuffer.addSample(measurement.timestamp.in(Seconds), odometryPose);
        Twist2d odometryTwistFromLastPose = lastOdometryPose.log(odometryPose);
        estimatedPose = estimatedPose.exp(odometryTwistFromLastPose);
        fieldsTable.recordOutput("Current Estimated Pose", estimatedPose);
        fieldsTable.recordOutput("Odometry to Estimated Transform (Odometry error)", new Transform2d(odometryPose, estimatedPose));
        callAllCallbacks();
    }

    public void addVisionMeasurement(VisionMeasurement measurement) {
        if (measurement.timestamp.lt(lastResetTimestamp)) return;
        try {
            Time dt = measurement.timestamp.minus(Seconds.of(odometryPosesBuffer.getInternalBuffer().lastKey()));
            if (dt.gt(ODOMETRY_POSES_BUFFER_SIZE)) {
                return;
            }
        } catch (NoSuchElementException ex) {
            return;
        }
        if (DriverStation.isDisabled()) {
            measurement = new VisionMeasurement(measurement.pose, measurement.trustLevel.multiply(preMatchVisionTrustLevelMultiplier.get()), measurement.timestamp);
        }
        Optional<Pose2d> sample = odometryPosesBuffer.getSample(measurement.timestamp().in(Seconds));
        if (sample.isEmpty())
            return;
        Transform2d odometryToSampleTransform = new Transform2d(odometryPose, sample.get());
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);
        Transform2d visionTransform = calculateVisionTransform(measurement, estimateAtTime);
        estimatedPose = estimateAtTime.plus(visionTransform).plus(odometryToSampleTransform.inverse());
        fieldsTable.recordOutput("Current Estimated Pose", estimatedPose);
        fieldsTable.recordOutput("Odometry to Estimated Transform (Odometry error)", new Transform2d(odometryPose, estimatedPose));
        callAllCallbacks();
    }

    private Transform2d calculateVisionTransform(VisionMeasurement visionMeasurement, Pose2d estimateAtTime) {
        // Closed-form Kalman gain for a continuous Kalman filter with A = 0 and C = I
        // (see wpimath/algorithms.md): k = q / (q + sqrt(q * r)), with q = σ_odom², r = σ_vision².
        // This simplifies to k = 1 / (1 + σ_vision / σ_odom), so the gain depends only on the
        // ratio of the std devs (not linearly): k → 1 as odometry trust drops, k → 0 as vision trust drops.
        double[] r = trustLevelToArraySquared(visionMeasurement.trustLevel);
        double[] q = trustLevelToArraySquared(this.odometryTrustLevel.get());
        Matrix<N3, N3> visionK = new Matrix<N3, N3>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; row++) {
            if (q[row] == 0) {
                visionK.set(row, row, 0);
            } else {
                visionK.set(row, row,
                        q[row] / (q[row] + Math.sqrt(q[row] * r[row])));
            }
        }

        Transform2d transform = new Transform2d(estimateAtTime, visionMeasurement.pose());

        Matrix<N3, N1> kTimesTransform = visionK.times(
                VecBuilder.fill(transform.getX(), transform.getY(), transform.getRotation().getRadians()));

        return new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0)));
    }

    private static double[] trustLevelToArraySquared(TrustLevel trustLevel) {
        double[] arr = new double[3];
        arr[0] = Math.pow(trustLevel.xyStdDev(), 2);
        arr[1] = Math.pow(trustLevel.xyStdDev(), 2);
        arr[2] = Math.pow(trustLevel.rotationStdDev(), 2);
        return arr;
    }

    private void callAllCallbacks() {
        for (Consumer<Pose2d> callback : callbackOnPoseUpdate) {
            callback.accept(estimatedPose);
        }
    }

    public static void registerCallbackOnPoseUpdate(Consumer<Pose2d> callback) {
        callbackOnPoseUpdate.add(callback);
    }

    public void resetPose(Pose2d newPose) {
        odometryPose = newPose;
        estimatedPose = newPose;
        odometryPosesBuffer.clear();
        lastGyroAngle = Optional.empty();
        yawRotationalSensorHelper.resetAngle(0);
        lastResetTimestamp = Seconds.of(Timer.getTimestamp());
        fieldsTable.recordOutput("Current Odometry Pose", odometryPose);
        fieldsTable.recordOutput("Current Estimated Pose", estimatedPose);
        fieldsTable.recordOutput("Odometry to Estimated Transform (Odometry error)", new Transform2d(odometryPose, estimatedPose));
        callAllCallbacks();
    }

    public void resetYaw(Rotation2d newYaw) {
        resetPose(new Pose2d(estimatedPose.getTranslation(), newYaw));
    }

    public void resetYawZero() {
        resetYaw(Rotation2d.fromDegrees(RobotContainer.isRedAlliance() ? 0 : 180));
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addChild("Odometry trust level", this.odometryTrustLevel);
        builder.addChild("Pre match vision trust level multiplier", preMatchVisionTrustLevelMultiplier);
    }

    public record VisionMeasurement(Pose2d pose, TrustLevel trustLevel, Time timestamp) {
    }

    public record OdometryMeasurement(
        Time timestamp,
        SwerveDriveKinematics kinematics, SwerveModulePosition[] modulePositionsDelta,
        SwerveModuleState[] moduleStates,
        Translation2d robotIMUAcceleration,
        Optional<Rotation2d> gyroAngle
        ) {
    }
}
