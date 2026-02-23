package frc.robot.subsystems.poseestimation;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.poseestimation.CollisionDetector.CollisionDetectorInfo;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.TunablesManager;

import static frc.robot.subsystems.poseestimation.PoseEstimatorConstants.*;

public class PoseEstimator {
    private static final PoseEstimator instance = new PoseEstimator();
    private static final List<Consumer<Pose2d>> callbackOnPoseUpdate = new ArrayList<>();

    private Pose2d odomertryPose = Pose2d.kZero;
    private Pose2d estimatedPose = Pose2d.kZero;

    private final TimeInterpolatableBuffer<Pose2d> odometryPosesBuffer = TimeInterpolatableBuffer
            .createBuffer(ODOMETRY_POSES_BUFFER_SIZE_SEC);

    private final LogFieldsTable fieldsTable = new LogFieldsTable("PoseEstimator");

    private final CollisionDetector collisionDetector = new CollisionDetector(fieldsTable.getSubTable("Collision Detector"));

    private final Debouncer inCollisionDebouncer = new Debouncer(IN_COLLISION_DEBOUNCE_SEC);

    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
    };

    private PoseEstimator() {
        TunablesManager.add("Collision Detector", collisionDetector);
    }

    public static PoseEstimator getInstance() {
        return instance;
    }

    public void updateCollision(CollisionDetectorInfo collisionInfo) {
        collisionDetector.update(collisionInfo);
    }

    public void addOdometryMeasurment(OdometryMeasurment measurment) {
        if (collisionDetector.inCollision())
            return;
        Twist2d twist2d = measurment.kinematics.toTwist2d(lastModulePositions, measurment.modulePositions);
        lastModulePositions = measurment.modulePositions;
        Pose2d lastOdometryPose = odomertryPose;
        odomertryPose = odomertryPose.exp(twist2d);
        if (measurment.gyroAngle.isPresent()) {
            odomertryPose = new Pose2d(odomertryPose.getTranslation(), measurment.gyroAngle.get());
        }
        fieldsTable.recordOutput("Current Odomertry Pose", odomertryPose);
        odometryPosesBuffer.addSample(measurment.timestamp, odomertryPose);
        Twist2d odometryTwistFromLastPose = lastOdometryPose.log(odomertryPose);
        estimatedPose = estimatedPose.exp(odometryTwistFromLastPose);
        fieldsTable.recordOutput("Current Estimated Pose", estimatedPose);
        callAllCallbacks();
    }

    public void addVisionMeasurment(VisionMesurment mesurment) {
        Optional<Pose2d> sample = odometryPosesBuffer.getSample(mesurment.timestamp());
        if (sample.isEmpty())
            return;
        Transform2d odometryToSampleTransform = new Transform2d(odomertryPose, sample.get());
        Pose2d estimateAtTime = estimatedPose.plus(odometryToSampleTransform);
        Transform2d visionTransform = calculateVisionTransform(mesurment, estimateAtTime);
        estimatedPose = estimateAtTime.plus(visionTransform).plus(odometryToSampleTransform.inverse());
        fieldsTable.recordOutput("Current Estimated Pose", estimatedPose);
        callAllCallbacks();
    }

    private Transform2d calculateVisionTransform(VisionMesurment visionMesurment, Pose2d estimateAtTime) {
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md
        double[] r = new double[] { visionMesurment.xyStdDev(), visionMesurment.xyStdDev(),
                visionMesurment.thetaStdDev() };

        Matrix<N3, N3> visionK = new Matrix<N3, N3>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; row++) {
            if (VISION_Q_STD_DEVS[row] == 0) {
                visionK.set(row, row, 0);
            } else {
                visionK.set(row, row,
                        VISION_Q_STD_DEVS[row] / (VISION_Q_STD_DEVS[row] + Math.sqrt(VISION_Q_STD_DEVS[row] * r[row])));
            }
        }

        Transform2d transform = new Transform2d(estimateAtTime, visionMesurment.pose());

        Matrix<N3, N1> kTimesTransform = visionK.times(
                VecBuilder.fill(transform.getX(), transform.getY(), transform.getRotation().getRadians()));

        return new Transform2d(
                kTimesTransform.get(0, 0),
                kTimesTransform.get(1, 0),
                Rotation2d.fromRadians(kTimesTransform.get(2, 0)));
    }

    public boolean inCollision() {
        return inCollisionDebouncer.calculate(collisionDetector.inCollision());
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
        odomertryPose = newPose;
        estimatedPose = newPose;
        odometryPosesBuffer.clear();
        fieldsTable.recordOutput("Current Odomertry Pose", odomertryPose);
        fieldsTable.recordOutput("Current Estimated Pose", estimatedPose);
        callAllCallbacks();
    }

    public void resetYaw(Rotation2d newYaw) {
        resetPose(new Pose2d(estimatedPose.getTranslation(), newYaw));
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public Pose2d getOdometryPose() {
        return odomertryPose;
    }

    public record VisionMesurment(Pose2d pose, double xyStdDev, double thetaStdDev, double timestamp) {
    }

    public record OdometryMeasurment(SwerveDriveKinematics kinematics, SwerveModulePosition[] modulePositions,
            Optional<Rotation2d> gyroAngle, double timestamp) {
    }
}
