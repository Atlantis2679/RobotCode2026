package frc.robot.subsystems.swerve;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotMap.ModuleFL;
import frc.robot.RobotMap.ModuleFR;
import frc.robot.RobotMap.ModuleBL;
import frc.robot.RobotMap.ModuleBR;
import frc.robot.subsystems.poseestimation.CollisionDetector.CollisionDetectorInfo;
import frc.robot.subsystems.poseestimation.PoseEstimator;
import frc.robot.subsystems.poseestimation.PoseEstimator.OdometryMeasurment;
import frc.robot.subsystems.swerve.SwerveConstants.Modules;
import frc.robot.subsystems.swerve.io.ImuIO;
import frc.robot.subsystems.swerve.io.ImuIONavX;
import frc.robot.subsystems.swerve.io.ImuIOSim;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.valueholders.DoubleHolder;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import java.util.Optional;

public class Swerve extends SubsystemBase implements Tunable {
  private LogFieldsTable fieldsTable = new LogFieldsTable(getName());

  private SwerveModule[] modules = {
      new SwerveModule(fieldsTable, 0, ModuleFL.DRIVE_MOTOR_ID, ModuleFL.TURN_MOTOR_ID, ModuleFL.CAN_CODER_ID),
      new SwerveModule(fieldsTable, 1, ModuleFR.DRIVE_MOTOR_ID, ModuleFR.TURN_MOTOR_ID, ModuleFR.CAN_CODER_ID),
      new SwerveModule(fieldsTable, 2, ModuleBL.DRIVE_MOTOR_ID, ModuleBL.TURN_MOTOR_ID, ModuleBL.CAN_CODER_ID),
      new SwerveModule(fieldsTable, 3, ModuleBR.DRIVE_MOTOR_ID, ModuleBR.TURN_MOTOR_ID, ModuleBR.CAN_CODER_ID),
  };

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(MODULES_LOCATIONS);

  private final RotationalSensorHelper gyroYawDegreesCCW;

  private final ImuIO imuIO = Robot.isReal() ? new ImuIONavX(fieldsTable) : new ImuIOSim(fieldsTable);

  private final Debouncer isGyroConnectedDebouncer = new Debouncer(GYRO_CONNECTED_DEBUNCER_SECONDS);

  public Swerve() {
    fieldsTable.update();

    TunablesManager.add("Swerve", (Tunable) this);

    gyroYawDegreesCCW = new RotationalSensorHelper(imuIO.angleDegreesCCW.getAsDouble());
    gyroYawDegreesCCW.enableContinuousWrap(0, 360);

    PeriodicAlertsGroup.defaultInstance.addErrorAlert(() -> "Gyro Disconnected!", () -> !isGyroConnected());

    resetGyroYawZero();
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.periodic();
    }

    gyroYawDegreesCCW.update(imuIO.angleDegreesCCW.getAsDouble());

    Optional<Rotation2d> gyroAngle = isGyroConnected() ? Optional.of(Rotation2d.fromDegrees(getGyroYawDegreesCCW()))
        : Optional.empty();
    PoseEstimator.getInstance().updateCollision(
        new CollisionDetectorInfo(getXAcceleration(), getYAcceleration(), getZAcceleration(), getModulesCurrents()));
    PoseEstimator.getInstance().addOdometryMeasurment(
        new OdometryMeasurment(kinematics, getModulePositions(), gyroAngle, Timer.getFPGATimestamp()));

    fieldsTable.recordOutput("Is gryo connected", isGyroConnected());
    fieldsTable.recordOutput("Yaw degrees CCW", getGyroYawDegreesCCW());
    fieldsTable.recordOutput("Current Command", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");
  }

  public void drive(double vxSpeedMPS, double vySpeedMPS, double vAngleRandiansPS, boolean isFieldRelative,
      boolean useVoltage) {
    fieldsTable.recordOutput("vxSpeedMPS", vxSpeedMPS);
    fieldsTable.recordOutput("vySpeedMPS", vySpeedMPS);
    fieldsTable.recordOutput("isFieldRelative", isFieldRelative);
    ChassisSpeeds targetChassisSpeeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        RobotContainer.isRedAlliance() ? -vxSpeedMPS : vxSpeedMPS,
        RobotContainer.isRedAlliance() ? -vySpeedMPS : vySpeedMPS,
        vAngleRandiansPS, PoseEstimator.getInstance().getEstimatedPose().getRotation())
        : new ChassisSpeeds(vxSpeedMPS, vySpeedMPS, vAngleRandiansPS);

    driveChassisSpeeds(targetChassisSpeeds, useVoltage);
  }

  public void stop() {
    drive(0, 0, 0, false, true);
  }

  public double getGyroYawDegreesCCW() {
    return gyroYawDegreesCCW.getAngle();
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
    for (int i = 0; i < modules.length; i++) {
      positions[i] = modules[i].getModulePosition();
    }
    return positions;
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return kinematics.toChassisSpeeds(
        modules[0].getModuleState(),
        modules[1].getModuleState(),
        modules[2].getModuleState(),
        modules[3].getModuleState());
  }

  public boolean isGyroConnected() {
    return isGyroConnectedDebouncer.calculate(imuIO.isConnected.getAsBoolean());
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds, boolean useVoltage) {
    fieldsTable.recordOutput("Modules Target Chassis Speeds", speeds);

    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Modules.MAX_SPEED_MPS);

    setModulesState(swerveModuleStates, true, true, useVoltage);
  }

  public void resetGyroYaw(double newAngleDegreesCCW) {
    gyroYawDegreesCCW.resetAngle(newAngleDegreesCCW);
  }

  public void resetGyroYawZero() {
    resetGyroYaw(RobotContainer.isRedAlliance() ? 0 : 180);
  }

  public void resetModulesToAbsoulte() {
    for (SwerveModule module : modules)
      module.resetIntegratedAngleToAbsolute();
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

  public double getXAcceleration() {
    return imuIO.xAcceleration.getAsDouble();
  }

  public double getYAcceleration() {
    return imuIO.yAcceleration.getAsDouble();
  }

  public double getZAcceleration() {
    return imuIO.zAcceleration.getAsDouble();
  }

  public double[] getModulesCurrents() {
    double[] currents = new double[4];
    for (int i = 0; i < 4; ++i) {
      currents[i] = Math.abs(modules[i].getCurrent());
    }
    return currents;
  }

  @Override
  public void initTunable(TunableBuilder builder) {
    builder.addChild("Swerve Subsystem", (Sendable) this);

    builder.addChild("Module 0 FL", modules[0]);
    builder.addChild("Module 1 FR", modules[1]);
    builder.addChild("Module 2 BL", modules[2]);
    builder.addChild("Module 3 BR", modules[3]);

    builder.addChild("Reset moudles to absoulte",
        new InstantCommand(this::resetModulesToAbsoulte).ignoringDisable(true));

    builder.addChild("Reset absolute angle", (Tunable) (resetBuilder) -> {
      DoubleHolder angleToReset = new DoubleHolder(0);
      resetBuilder.addDoubleProperty("Angle to reset", angleToReset::get, angleToReset::set);
      resetBuilder.addChild("reset!", new InstantCommand(() -> {
        for (SwerveModule module : modules) {
          module.resetAngleDegreesCCW(angleToReset.get());
        }
      }).ignoringDisable(true));
    });
  }
}
