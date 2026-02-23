package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.Modules.MAX_SPEED_MPS;
import static frc.robot.subsystems.swerve.SwerveConstants.Modules.MAX_VOLTAGE;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.BooleanHolder;

public class SwerveCommands {
  private final Swerve swerve;

  public SwerveCommands(Swerve swerve) {
    this.swerve = swerve;
  }

  public TunableCommand driverController(DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier,
      DoubleSupplier rotationSupplier, DoubleSupplier yawAutoRotationSupplier,
      BooleanSupplier autoRotationMode, BooleanSupplier isFieldRelativeSupplier, BooleanSupplier isSensetiveMode) {

    return new SwerveDriverController(swerve, forwardSupplier, sidewaysSupplier, rotationSupplier,
        yawAutoRotationSupplier, autoRotationMode,
        isFieldRelativeSupplier, isSensetiveMode);
  }

  public Command driveForward(DoubleSupplier forwardPrecentageSupplier) {
    return swerve.run(
      () -> swerve.drive(forwardPrecentageSupplier.getAsDouble() * MAX_VOLTAGE, 0.0, 0.0, false, true));
  }

  public Command xWheelLock() {
    return swerve.runOnce(() -> {
      SwerveModuleState[] moduleStates = new SwerveModuleState[4];
      moduleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
      moduleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
      moduleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(135));
      moduleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(-135));

      swerve.setModulesState(moduleStates, true, false, false);
    }).withName("wheel lock");
  }

  public TunableCommand controlModules(DoubleSupplier turnXSupplier, DoubleSupplier turnYSupplier,
      DoubleSupplier speedSupplier) {
    return TunableCommand.wrap(tuneableBuilder -> {
      BooleanHolder optimizeState = tuneableBuilder.addBoolean("Optimize State", true);
      return new RunCommand(() -> {
        double turnY = turnYSupplier.getAsDouble();
        double turnX = turnXSupplier.getAsDouble();
        double speed = speedSupplier.getAsDouble();

        SwerveModuleState[] moduleStates = new SwerveModuleState[4];
        for (int i = 0; i < moduleStates.length; i++) {

          double turnAngleRadians = turnX != 0 || turnY != 0
              ? Math.atan2(turnY, turnX) - Math.toRadians(90)
              : 0;

          moduleStates[i] = new SwerveModuleState(
              speed * MAX_SPEED_MPS,
              new Rotation2d(turnAngleRadians));
        }
        swerve.setModulesState(moduleStates, optimizeState.get(), false, false);
      }, swerve);
    });
  }

  public Command stop() {
    return swerve.run(swerve::stop).ignoringDisable(true).withName("stop");
  }
}