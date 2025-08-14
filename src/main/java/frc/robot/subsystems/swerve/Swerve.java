package frc.robot.subsystems.swerve;

import java.io.IOError;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.swerve.io.GyroIO;
import frc.robot.subsystems.swerve.io.GyroIONavX;
import frc.robot.subsystems.swerve.io.GyroIOSim;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;

import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.RobotMap.*;

public class Swerve extends SubsystemBase {
  private LogFieldsTable fieldsTable = new LogFieldsTable(getName());

  private SwerveModule[] modules = {
    new SwerveModule(fieldsTable.getSubTable(getModuleName(0)), 0, Module0.DRIVE_MOTOR_ID, Module0.TURN_MOTOR_ID, Module0.CAN_CODER_ID),
    new SwerveModule(fieldsTable.getSubTable(getModuleName(1)), 1, Module1.DRIVE_MOTOR_ID, Module1.TURN_MOTOR_ID, Module1.CAN_CODER_ID),
    new SwerveModule(fieldsTable.getSubTable(getModuleName(2)), 2, Module2.DRIVE_MOTOR_ID, Module2.TURN_MOTOR_ID, Module2.CAN_CODER_ID),
    new SwerveModule(fieldsTable.getSubTable(getModuleName(3)), 3, Module3.DRIVE_MOTOR_ID, Module3.TURN_MOTOR_ID, Module3.CAN_CODER_ID),
  };
  
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULES_LOCATIONS);

  private final RotationalSensorHelper yawDegreesCW = new RotationalSensorHelper(0);

  private final GyroIO gyroIO = Robot.isReal() ? new GyroIONavX(fieldsTable) : new GyroIOSim(fieldsTable);

  public Swerve() {
    fieldsTable.update();

    yawDegreesCW.enableContinuousWrap(0, 360);
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.periodic();
    }

    yawDegreesCW.update(gyroIO.angleDegreesCw.getAsDouble() * 360);
    fieldsTable.recordOutput("Gyro yaw degrees CW", getYawDegreesCW());
  }

  public void drive(double vxSpeedMPS, double vySpeedMPS, double vAngleRandiansPS, boolean isFieldRelative) {
    ChassisSpeeds targetChassisSpeeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(vxSpeedMPS, vySpeedMPS, vAngleRandiansPS, ) : ChassisSpeeds.fromRobotRelativeSpeeds(targetChassisSpeeds, null);
  }

  public double getYawDegreesCW() {
    return yawDegreesCW.getAngle();
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
