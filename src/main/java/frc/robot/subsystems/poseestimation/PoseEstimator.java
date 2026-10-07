package frc.robot.subsystems.poseestimation;

import static frc.robot.subsystems.poseestimation.PoseEstimatorConstants.PRE_MATCH_VISION_TRUST_LEVEL_MULTIPLIER;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.subsystems.poseestimation.PoseEstimatorConstants.ODOMETRY_POSES_BUFFER_SIZE;
import static frc.robot.subsystems.poseestimation.PoseEstimatorConstants.ODOMETRY_STD_DEVS;

import java.util.ArrayList;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.Interpolatable;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
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

    public record OdoemtrySample(Pose2d pose, double skidRatio) implements Interpolatable<OdoemtrySample> {
        @Override
        public OdoemtrySample interpolate(OdoemtrySample endValue, double t) {
            Pose2d interpolatedPose = this.pose.interpolate(endValue.pose, t);
            double interpolatedSkidRatio = MathUtil.interpolate(this.skidRatio, endValue.skidRatio, t);
            return new OdoemtrySample(interpolatedPose, interpolatedSkidRatio);
        }
    }

    private final TimeInterpolatableBuffer<OdoemtrySample> odometryPosesBuffer =
        TimeInterpolatableBuffer.createBuffer(ODOMETRY_POSES_BUFFER_SIZE.in(Seconds));
    
    private final LogFieldsTable fieldsTable = new LogFieldsTable("PoseEstimator");

    private final SkidDetector skidDetector = new SkidDetector(fieldsTable.getSubTable("SkidDetector"));
    
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
        yawRotationalSensorHelper.update(measurement.gyroAngle.get().getDegrees());
        Pose2d lastOdometryPose = odometryPose;
        odometryPose = odometryPose.exp(twist2d);
        if (measurement.gyroAngle.isPresent()) {
            odometryPose = new Pose2d(odometryPose.getTranslation(), Rotation2d.fromDegrees(yawRotationalSensorHelper.getAngle()));
        }
        fieldsTable.recordOutput("Current Odometry Pose", odometryPose);
        double skidRatio = skidDetector.update(
            measurement.timestamp,
            measurement.moduleStates,
            Rotation2d.fromDegrees(yawRotationalSensorHelper.getAngle()),
            DegreesPerSecond.of(yawRotationalSensorHelper.getVelocity()),
            measurement.robotIMUAcceleration,
            RobotState.isDisabled()
            );
        odometryPosesBuffer.addSample(measurement.timestamp.in(Seconds), new OdoemtrySample(odometryPose, skidRatio));
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
        Optional<OdoemtrySample> sample = odometryPosesBuffer.getSample(measurement.timestamp().in(Seconds));
        if (sample.isEmpty())
            return;
        Transform2d odometryToSampleTransform = new Transform2d(odometryPose, sample.get().pose);
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);
        Transform2d visionTransform = calculateVisionTransform(measurement, estimateAtTime, sample.get().skidRatio);
        estimatedPose = estimateAtTime.plus(visionTransform).plus(odometryToSampleTransform.inverse());
        fieldsTable.recordOutput("Current Estimated Pose", estimatedPose);
        fieldsTable.recordOutput("Odometry to Estimated Transform (Odometry error)", new Transform2d(odometryPose, estimatedPose));
        callAllCallbacks();
    }

    private Transform2d calculateVisionTransform(VisionMeasurement visionMeasurement, Pose2d estimateAtTime,
        double skidRatioAtTime) {
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md
        // Worth noting that this in it's current form may be simplefied to
        // k = 1 / (1 + q / r), meaning k is linearly proportional to the ratio between Q_STD and Vision STD
        TrustLevel skidTrustLevelMutliplier = new TrustLevel(Math.pow(2, skidRatioAtTime), Math.pow(2, skidRatioAtTime));
        TrustLevel odometryTrustLevel = this.odometryTrustLevel.get().multiply(skidTrustLevelMutliplier);
        double[] r = trustLevelToArraySquared(visionMeasurement.trustLevel);
        double[] q = trustLevelToArraySquared(odometryTrustLevel);
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
        builder.addChild("Vision Q", this.odometryTrustLevel);
        builder.addChild("No Odometry trust level multiplyer", preMatchVisionTrustLevelMultiplier);
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
