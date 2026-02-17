package frc.robot.subsystems.poseestimation;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.subsystems.poseestimation.CollisionDetector.CollisionDetectorInfo;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.tunables.TunablesManager;

public class PoseEstimator {
    private static final PoseEstimator instance = new PoseEstimator();

    private Pose2d odomertryPose = Pose2d.kZero;
    private Pose2d estimatedPose = Pose2d.kZero;

    private LogFieldsTable fieldsTable = new LogFieldsTable("PoseEstimator");

    private CollisionDetector collisionDetector = new CollisionDetector(fieldsTable);

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

    public void update(SwerveDriveKinematics kinematics, SwerveModulePosition[] modulePositions, Optional<Rotation2d> gyroAngle, CollisionDetectorInfo collisionDetectorInfo) {
        collisionDetector.check(collisionDetectorInfo);
        Twist2d twist2d = kinematics.toTwist2d(lastModulePositions, modulePositions);
        lastModulePositions = modulePositions;
        Pose2d lastOdometryPose = odomertryPose;
        odomertryPose = odomertryPose.exp(twist2d);
        if (gyroAngle.isPresent()) {
            odomertryPose = new Pose2d(odomertryPose.getTranslation(), gyroAngle.get());
        }
        fieldsTable.recordOutput("Current Odomertry Pose", odomertryPose);
        Twist2d twistFromLastPose = lastOdometryPose.log(odomertryPose);
        estimatedPose = estimatedPose.exp(twistFromLastPose);
        fieldsTable.recordOutput("Current Estimated Pose", estimatedPose);
    }

    public void resetPose(Pose2d newPose) {
        odomertryPose = newPose;
        estimatedPose = newPose;
        fieldsTable.recordOutput("Current Odomertry Pose", odomertryPose);
        fieldsTable.recordOutput("Current Estimated Pose", estimatedPose);
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
}