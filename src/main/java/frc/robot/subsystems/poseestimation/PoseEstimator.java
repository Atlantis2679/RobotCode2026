package frc.robot.subsystems.poseestimation;

import static frc.robot.subsystems.poseestimation.PoseEstimatorConstants.NO_ODOMETRY_TRUST_LEVEL_MULTIPLIER;
import static frc.robot.subsystems.poseestimation.PoseEstimatorConstants.ODOMETRY_POSES_BUFFER_SIZE_SEC;
import static frc.robot.subsystems.poseestimation.PoseEstimatorConstants.VISION_Q_STD_DEVS;

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
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotContainer;
import frc.robot.subsystems.vision.Vision.TrustLevel;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;

public class PoseEstimator implements Tunable {
    private static final List<Consumer<Pose2d>> callbackOnPoseUpdate = new ArrayList<>();
    private static final PoseEstimator instance = new PoseEstimator();

    private Pose2d odometryPose = Pose2d.kZero;
    private Pose2d estimatedPose = Pose2d.kZero;

    private final TimeInterpolatableBuffer<Pose2d> odometryPosesBuffer = TimeInterpolatableBuffer
            .createBuffer(ODOMETRY_POSES_BUFFER_SIZE_SEC);

    private final LogFieldsTable fieldsTable = new LogFieldsTable("PoseEstimator");
    
    private TrustLevel visionTrustLevelQ = VISION_Q_STD_DEVS;

    private TrustLevel noOdometryTrustLevelMultiplier = NO_ODOMETRY_TRUST_LEVEL_MULTIPLIER;

    private Rotation2d gyroOffset = new Rotation2d();
    private Rotation2d lastGyroAngle = new Rotation2d();

    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
    };

    private PoseEstimator() {}

    public static PoseEstimator getInstance() {
        return instance;
    }

    public void addOdometryMeasurement(OdometryMeasurement measurement) {
        Twist2d twist2d = measurement.kinematics.toTwist2d(lastModulePositions, measurement.modulePositions);
        lastModulePositions = measurement.modulePositions;
        Pose2d lastOdometryPose = odometryPose;
        odometryPose = odometryPose.exp(twist2d);
        if (measurement.gyroAngle.isPresent()) {
            lastGyroAngle = measurement.gyroAngle.get();
            odometryPose = new Pose2d(odometryPose.getTranslation(), measurement.gyroAngle.get().minus(gyroOffset));
        }
        fieldsTable.recordOutput("Current Odometry Pose", odometryPose);
        odometryPosesBuffer.addSample(measurement.timestamp, odometryPose);
        Twist2d odometryTwistFromLastPose = lastOdometryPose.log(odometryPose);
        estimatedPose = estimatedPose.exp(odometryTwistFromLastPose);
        fieldsTable.recordOutput("Current Estimated Pose", estimatedPose);
        fieldsTable.recordOutput("Odometry to Estimated Transform (Odometry error)", new Transform2d(odometryPose, estimatedPose));
        callAllCallbacks();
    }

    public void addVisionMeasurement(VisionMeasurement measurement) {
        try {
          if (odometryPosesBuffer.getInternalBuffer().lastKey() - ODOMETRY_POSES_BUFFER_SIZE_SEC > measurement.timestamp()) {
            return;
          }
        } catch (NoSuchElementException ex) {
          return;
        }
        if (DriverStation.isDisabled()) {
          measurement.trustLevel.multiply(noOdometryTrustLevelMultiplier);
        }
        Optional<Pose2d> sample = odometryPosesBuffer.getSample(measurement.timestamp());
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
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md
        double[] r = trustLevelToArraySquared(visionMeasurement.trustLevel);
        double[] q = trustLevelToArraySquared(visionTrustLevelQ);
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
        arr[0] = Math.pow(trustLevel.getXyStdDev(), 2);
        arr[1] = Math.pow(trustLevel.getXyStdDev(), 2);
        arr[2] = Math.pow(trustLevel.getRotationStdDev(), 2);
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
        gyroOffset = lastGyroAngle.minus(newPose.getRotation());
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
        builder.addChild("Vision Q", this.visionTrustLevelQ);
        builder.addChild("No Odometry trust level multiplyer", noOdometryTrustLevelMultiplier);
    }

    public record VisionMeasurement(Pose2d pose, TrustLevel trustLevel, double timestamp) {
    }

    public record OdometryMeasurement(SwerveDriveKinematics kinematics, SwerveModulePosition[] modulePositions,
            Optional<Rotation2d> gyroAngle, double timestamp) {
    }
}
