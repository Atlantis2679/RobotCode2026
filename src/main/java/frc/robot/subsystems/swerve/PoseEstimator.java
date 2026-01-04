package frc.robot.subsystems.swerve;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class PoseEstimator {
    private static final double[] qStdDevs = new double[] {0, 0, 0};
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

    public void update(SwerveModulePosition[] modulePositions, Optional<Rotation2d> gyroAngle, List<VisionMesurment> visionMesurments) {
        Twist2d twist2d = kinematics.toTwist2d(lastModulePositions, modulePositions);
        lastModulePositions = modulePositions;
        Pose2d lastOdometryPose = odomertryPose;
        odomertryPose = odomertryPose.exp(twist2d);
        if (gyroAngle.isPresent()) {
            odomertryPose = new Pose2d(odomertryPose.getTranslation(), gyroAngle.get());
        }
        fieldsTable.recordOutput("Current Odomertry Pose", odomertryPose);
        Twist2d odometryTwistFromLastPose = lastOdometryPose.log(odomertryPose);
        estimatedPose = estimatedPose.exp(odometryTwistFromLastPose);
        for (VisionMesurment visionMesurment : visionMesurments) {
            Transform2d visionTransformFromOdometryPose = calculateVisionTransform(visionMesurment, odomertryPose);
            estimatedPose.plus(visionTransformFromOdometryPose);
        }
        fieldsTable.recordOutput("Current Estimated Pose", estimatedPose);
    }

    private Transform2d calculateVisionTransform(VisionMesurment visionMesurment, Pose2d odometryPose) {
        // Solve for closed form Kalman gain for continuous Kalman filter with A = 0
        // and C = I. See wpimath/algorithms.md 
        double[] r = new double[3];
        for (int i = 0; i < 3; ++i) {
          r[i] = Math.pow(visionMesurment.stdDevs().get(i), 2);
        }    
        Matrix<N3, N3> visionK = new Matrix<N3, N3>(Nat.N3(), Nat.N3());
        for (int row = 0; row < 3; row++) {
            if (qStdDevs[row] == 0) {
                visionK.set(row, row, 0);
            } else {
                visionK.set(row, row, qStdDevs[row] / (qStdDevs[row] + Math.sqrt(qStdDevs[row] * r[row])));
            }
        }

        Transform2d transform = new Transform2d(odometryPose, visionMesurment.pose);

        Matrix<N3, N1> kTimesTransform = visionK.times(
            VecBuilder.fill(transform.getX(), transform.getY(), transform.getRotation().getRadians()));
        
        return new Transform2d(
            kTimesTransform.get(0, 0),
            kTimesTransform.get(1, 0),
            Rotation2d.fromRadians(kTimesTransform.get(2, 0)));
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

    public record VisionMesurment(Pose2d pose, Vector<N3> stdDevs) {}
}
