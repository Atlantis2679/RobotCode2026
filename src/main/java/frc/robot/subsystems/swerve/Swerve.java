package frc.robot.subsystems.swerve;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap.Module0;
import frc.robot.RobotMap.Module1;
import frc.robot.RobotMap.Module2;
import frc.robot.RobotMap.Module3;
import frc.robot.subsystems.swerve.io.GyroIO;
import frc.robot.subsystems.swerve.io.GyroIONavX;
import frc.robot.subsystems.swerve.io.GyroIOSim;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class Swerve extends SubsystemBase {
  private LogFieldsTable fieldsTable = new LogFieldsTable(getName());

  private SwerveModule[] modules = {
      new SwerveModule(fieldsTable, 0, Module0.DRIVE_MOTOR_ID, Module0.TURN_MOTOR_ID, Module0.CAN_CODER_ID),
      new SwerveModule(fieldsTable, 1, Module1.DRIVE_MOTOR_ID, Module1.TURN_MOTOR_ID, Module1.CAN_CODER_ID),
      new SwerveModule(fieldsTable, 2, Module2.DRIVE_MOTOR_ID, Module2.TURN_MOTOR_ID, Module2.CAN_CODER_ID),
      new SwerveModule(fieldsTable, 3, Module3.DRIVE_MOTOR_ID, Module3.TURN_MOTOR_ID, Module3.CAN_CODER_ID),
  };

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULES_LOCATIONS);

  private final RotationalSensorHelper yawDegreesCW = new RotationalSensorHelper(0);

  private final GyroIO gyroIO = Robot.isReal() ? new GyroIONavX(fieldsTable) : new GyroIOSim(fieldsTable);

  private final Debouncer isGyroConnectedDebouncer = new Debouncer(GYRO_CONNECTED_DEBUNCER_SECONDS);

  private final LoggedDashboardChooser<Boolean> isRedAlliance = new LoggedDashboardChooser<>("alliance");

  public Swerve() {
    fieldsTable.update();

    isRedAlliance.addDefaultOption("blue", false);
    isRedAlliance.addOption("red", true);

    yawDegreesCW.enableContinuousWrap(0, 360);

    PeriodicAlertsGroup.defaultInstance.addErrorAlert(() -> "Gyro Disconnected!", () -> !isGyroConnected());
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.periodic();
    }
    if (isGyroConnected()) {
      yawDegreesCW.update(gyroIO.angleDegreesCw.getAsDouble() * 360);
    } else {
      Twist2d twist = kinematics.toTwist2d(
          modules[0].getModulePositionDelta(),
          modules[1].getModulePositionDelta(),
          modules[2].getModulePositionDelta(),
          modules[3].getModulePositionDelta());

      yawDegreesCW.update(twist.dtheta);
    }

    fieldsTable.recordOutput("Is gryo connected", isGyroConnected());
    fieldsTable.recordOutput("Yaw degrees CW", getYawDegreesCW());
    fieldsTable.recordOutput("Current Command", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

    SmartDashboard.putBoolean("isRedAlliance", isRedAlliance());
  }

  public void drive(double vxSpeedMPS, double vySpeedMPS, double vAngleRandiansPS, boolean isFieldRelative,
      boolean useVoltage) {
    ChassisSpeeds targetChassisSpeeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        isRedAlliance() ? -vxSpeedMPS : vxSpeedMPS,
        isRedAlliance() ? -vySpeedMPS : vySpeedMPS,
        vAngleRandiansPS, Rotation2d.fromDegrees(-getYawDegreesCW()))
        : new ChassisSpeeds(vxSpeedMPS, vySpeedMPS, vAngleRandiansPS);

    driveChassisSpeeds(targetChassisSpeeds, useVoltage);
  }

  public void stop() {
    drive(0, 0, 0, false, true);
  }

  public double getYawDegreesCW() {
    return yawDegreesCW.getAngle();
  }

  public boolean isRedAlliance() {
    return isRedAlliance.get().booleanValue();
  }

  public boolean isGyroConnected() {
    return isGyroConnectedDebouncer.calculate(gyroIO.isConnected.getAsBoolean());
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds, boolean useVoltage) {
    fieldsTable.recordOutput("Modules Target Chassis Speeds", speeds);

    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Modules.MAX_SPEED_MPS);

    setModulesState(swerveModuleStates, true, false, useVoltage);
  }

  public void setModulesState(SwerveModuleState[] moduleStates, boolean optimize, boolean preventJittering,
      boolean useVoltage) {
    fieldsTable.recordOutput("Modules Target States", moduleStates);

    for (SwerveModule module : modules)
      module.setTargetState(moduleStates[module.getModuleNumber()], optimize, preventJittering, useVoltage);
  }

  public void costAll() {
    for (SwerveModule module : modules) {
      module.setCoast();
    }
  }
}
