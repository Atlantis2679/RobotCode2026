package frc.robot.subsystems.swerve;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class PoseEstimator {
    private Pose2d odomertryPose = Pose2d.kZero;
    private Pose2d estimatedPose = Pose2d.kZero;
    private SwerveDriveKinematics kinematics;

    private LogFieldsTable fieldsTable;

    private SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
    };

    public PoseEstimator(LogFieldsTable fieldsTable, SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
        this.fieldsTable = fieldsTable;
    }

    public void update(SwerveModulePosition[] modulePositions, Optional<Rotation2d> gyroAngle) {
        Twist2d twist2d = kinematics.toTwist2d(lastModulePositions, modulePositions);
        fieldsTable.recordOutput("Twist", twist2d);
        lastModulePositions = modulePositions;
        Pose2d lastOdometryPose = new Pose2d();
        odomertryPose.exp(twist2d);
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

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public Pose2d getOdometryPose() {
        return odomertryPose;
    }
}
