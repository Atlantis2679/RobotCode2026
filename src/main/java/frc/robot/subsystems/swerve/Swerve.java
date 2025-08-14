package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve.SwerveConstants.getModuleName;

import java.io.IOError;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.swerve.SwerveConstants.*;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Swerve extends SubsystemBase {
  private LogFieldsTable fieldsTable = new LogFieldsTable(getName());

  private SwerveModule[] modules = {
    new SwerveModule(fieldsTable.getSubTable(getModuleName(0)), 0, Moudle0.DRIVE_MOTOR_ID, Moudle0.TURN_MOTOR_ID, Moudle0.CAN_CODER_ID),
    new SwerveModule(fieldsTable.getSubTable(getModuleName(1)), 1, Moudle1.DRIVE_MOTOR_ID, Moudle1.TURN_MOTOR_ID, Moudle1.CAN_CODER_ID),
    new SwerveModule(fieldsTable.getSubTable(getModuleName(2)), 2, Moudle2.DRIVE_MOTOR_ID, Moudle2.TURN_MOTOR_ID, Moudle2.CAN_CODER_ID),
    new SwerveModule(fieldsTable.getSubTable(getModuleName(3)), 3, Moudle3.DRIVE_MOTOR_ID, Moudle3.TURN_MOTOR_ID, Moudle3.CAN_CODER_ID),
  };

  private static final Translation2d[] modulesLocations = {Moudle0.LOCATION, Moudle1.LOCATION, Moudle2.LOCATION, Moudle3.LOCATION};
  
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(modulesLocations);

  private final RotationalSensorHelper yawDegreesCW = new RotationalSensorHelper(0);

  private final GyroIO io = ;

  public Swerve() {
    fieldsTable.update();

    yawDegreesCW.enableContinuousWrap(0, 360);
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.periodic();
    }

    yawDegreesCW.update(gyroIO);
  }

  public void drive(double vxSpeedMPS, double vySpeedMPS, double vAngleRandiansPS, boolean isFieldRelative) {
    ChassisSpeeds targetChassisSpeeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vxSpeedMPS, vySpeedMPS, vAngleRandiansPS, ) : ChassisSpeeds.fromRobotRelativeSpeeds(targetChassisSpeeds, null);
  }

  public double getYaw() {
    return 
  }

  public void driveChassisSpeed(ChassisSpeeds speeds, boolean useVoltage) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_SPEED_MPS);
    
    setModulesState(swerveModuleStates, true, useVoltage);
  }

  public void setModulesState(SwerveModuleState[] moduleStates, boolean optimize, boolean useVoltage) {
    for (SwerveModule module : modules)
      module.setTargetState(moduleStates[module.getModuleNumber()], optimize, useVoltage);
  }
}
