package frc.robot.subsystems.swerve;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotMap.ModuleFL;
import frc.robot.RobotMap.ModuleFR;
import frc.robot.RobotMap.ModuleBL;
import frc.robot.RobotMap.ModuleBR;
import frc.robot.subsystems.swerve.SwerveConstants.Modules;
import frc.robot.subsystems.swerve.SwerveConstants.PathPlanner;
import frc.robot.subsystems.swerve.io.GyroIO;
import frc.robot.subsystems.swerve.io.GyroIONavX;
import frc.robot.subsystems.swerve.io.GyroIOSim;
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.valueholders.DoubleHolder;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

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

  private final GyroIO gyroIO = Robot.isReal() ? new GyroIONavX(fieldsTable) : new GyroIOSim(fieldsTable);

  private final Debouncer isGyroConnectedDebouncer = new Debouncer(GYRO_CONNECTED_DEBUNCER_SECONDS);

  private final LoggedDashboardChooser<Boolean> isRedAlliance = new LoggedDashboardChooser<>("alliance");

  private final PoseEstimator poseEstimator = new PoseEstimator(fieldsTable.getSubTable("poseEstimator"), kinematics);

  public final Translation2d FL_LOCATION = new Translation2d(
      TRACK_LENGTH_METERS / 2,
      TRACK_WIDTH_METERS / 2);
  public final Translation2d FR_LOCATION = new Translation2d(
      TRACK_LENGTH_METERS / 2,
      -TRACK_WIDTH_METERS / 2);
  public final Translation2d BL_LOCATION = new Translation2d(
      -TRACK_LENGTH_METERS / 2,
      TRACK_WIDTH_METERS / 2);
  public final Translation2d BR_LOCATION = new Translation2d(
      -TRACK_LENGTH_METERS / 2,
      -TRACK_WIDTH_METERS / 2);

  public Swerve() {
    fieldsTable.update();

    TunablesManager.add("Swerve", (Tunable) this);

    isRedAlliance.addDefaultOption("red", true);
    isRedAlliance.addOption("blue", false);

    isRedAlliance.getSendableChooser().onChange((str) -> {
      resetYaw(str == "red" ? 0 : 180);
    });

    gyroYawDegreesCCW = new RotationalSensorHelper(gyroIO.angleDegreesCCW.getAsDouble());
    gyroYawDegreesCCW.enableContinuousWrap(0, 360);

    PeriodicAlertsGroup.defaultInstance.addErrorAlert(() -> "Gyro Disconnected!", () -> !isGyroConnected());

    resetYaw(isRedAlliance() ? 0 : 180);

    ModuleConfig moduleConfig = new ModuleConfig(
        PathPlanner.WHEEL_RADIUS_METERS,
        PathPlanner.MAX_MODULE_VELOCITY_MPS,
        PathPlanner.FRICTION_WITH_CARPET,
        DCMotor.getFalcon500(1).withReduction(PathPlanner.GEAR_RATIO_DRIVE),
        130,
        1);

    RobotConfig robotConfig = new RobotConfig(PathPlanner.ROBOT_MASS_KG, PathPlanner.MOMENT_OF_INERTIA,
        moduleConfig, FL_LOCATION, FR_LOCATION, BL_LOCATION, BR_LOCATION);
    robotConfig.hasValidConfig();

    AutoBuilder.configure(
        this::getPose,
        this::resetPose,
        this::getRobotRelativeChassisSpeeds,
        (speeds, feedforward) -> driveChassisSpeeds(speeds, true),
        new PPHolonomicDriveController(
            new PIDConstants(
                PathPlanner.TRANSLATION_KP,
                PathPlanner.TRANSLATION_KI,
                PathPlanner.TRANSLATION_KD),
            new PIDConstants(
                PathPlanner.ROTATION_KP,
                PathPlanner.ROTATION_KI,
                PathPlanner.ROTATION_KD)),
        robotConfig,
        this::isRedAlliance,
        this);
  }

  @Override
  public void periodic() {
    for (SwerveModule module : modules) {
      module.periodic();
    }

    gyroYawDegreesCCW.update(gyroIO.angleDegreesCCW.getAsDouble());

    Optional<Rotation2d> gyroAngle = isGyroConnected() ? Optional.of(Rotation2d.fromDegrees(getGyroYawDegreesCCW()))
        : Optional.empty();
    poseEstimator.update(getModulePositions(), gyroAngle);

    fieldsTable.recordOutput("Is gryo connected", isGyroConnected());
    fieldsTable.recordOutput("Yaw degrees CCW", getGyroYawDegreesCCW());
    fieldsTable.recordOutput("Current Command", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");

    fieldsTable.recordOutput("Is red alliance", isRedAlliance());
    SmartDashboard.putBoolean("isRedAlliance", isRedAlliance());
  }

  public void drive(double vxSpeedMPS, double vySpeedMPS, double vAngleRandiansPS, boolean isFieldRelative,
      boolean useVoltage) {
    fieldsTable.recordOutput("vxSpeedMPS", vxSpeedMPS);
    fieldsTable.recordOutput("vySpeedMPS", vySpeedMPS);
    fieldsTable.recordOutput("isFieldRelative", isFieldRelative);
    ChassisSpeeds targetChassisSpeeds = isFieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        isRedAlliance() ? -vxSpeedMPS : vxSpeedMPS,
        isRedAlliance() ? -vySpeedMPS : vySpeedMPS,
        vAngleRandiansPS, getPose().getRotation())
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

  public boolean isRedAlliance() {
    return isRedAlliance.get() != null && isRedAlliance.get();
  }

  public boolean isGyroConnected() {
    return isGyroConnectedDebouncer.calculate(gyroIO.isConnected.getAsBoolean());
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds, boolean useVoltage) {
    fieldsTable.recordOutput("Modules Target Chassis Speeds", speeds);

    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Modules.MAX_SPEED_MPS);

    setModulesState(swerveModuleStates, true, true, useVoltage);
  }

  public void resetYaw(double newAngleDegreesCCW) {
    gyroYawDegreesCCW.resetAngle(newAngleDegreesCCW);
    Pose2d newPose = new Pose2d(getPose().getTranslation(), Rotation2d.fromDegrees(getGyroYawDegreesCCW()));
    poseEstimator.resetPose(newPose);
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

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPose();
  }

  public void resetPose(Pose2d pose2d) {
    poseEstimator.resetPose(pose2d);
  }

  public ChassisSpeeds getRobotRelativeChassisSpeeds() {
    return kinematics.toChassisSpeeds(
        modules[0].getModuleState(),
        modules[1].getModuleState(),
        modules[2].getModuleState(),
        modules[3].getModuleState());
  }

  public void costAll() {
    for (SwerveModule module : modules) {
      module.setCoast();
    }
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

    builder.addChild("Reset Yaw", (Tunable) (resetBuilder) -> {
      DoubleHolder angleToReset = new DoubleHolder(0);
      resetBuilder.addDoubleProperty("Angle to reset", angleToReset::get, angleToReset::set);
      resetBuilder.addChild("reset!", new InstantCommand(() -> resetYaw(angleToReset.get())).ignoringDisable(true));
    });
  }
}
