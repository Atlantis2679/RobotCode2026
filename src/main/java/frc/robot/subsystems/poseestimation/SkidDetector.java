package frc.robot.subsystems.poseestimation;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class SkidDetector {
    private final LogFieldsTable fieldsTable;

    public SkidDetector(LogFieldsTable fieldsTable) {
        this.fieldsTable = fieldsTable;
    }

    public double update(SwerveDriveKinematics kinematics, SwerveModuleState[] moduleStates) {
        ChassisSpeeds speeds = kinematics.toChassisSpeeds(moduleStates);
        double omega = speeds.omegaRadiansPerSecond;
        SwerveModuleState[] rotationalOnly = kinematics.toSwerveModuleStates(
            new ChassisSpeeds(0, 0, omega)
        );
        Translation2d[] translationalVelocities = new Translation2d[moduleStates.length];
        for (int i = 0; i < moduleStates.length; i++) { 
            Translation2d measured = new Translation2d(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle);
            Translation2d rotational = new Translation2d(rotationalOnly[i].speedMetersPerSecond, rotationalOnly[i].angle);
            Translation2d translational = measured.minus(rotational);
            translationalVelocities[i] = translational;
        }
        
        double skidRatio = 1;
        fieldsTable.recordOutput("Skid Ratio", skidRatio);
        return skidRatio;
    }
}
