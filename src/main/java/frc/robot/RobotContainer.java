package frc.robot;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.allCommands.AllCommands;
import frc.robot.allCommands.AllCommandsConstants;
import frc.robot.shooting.ShootingCalculator;
import frc.robot.shooting.ShootingMeasurments;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.fourbar.Fourbar;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utils.NaturalXboxController;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Fourbar fourbar = new Fourbar();
    private final Roller roller = new Roller();
    private final Index index = new Index();
    private final Hood hood = new Hood();
    private final FlyWheel flyWheel = new FlyWheel();
    private final Elevator elevator = new Elevator();

    private final ShootingCalculator hubShootingCalculator = new ShootingCalculator(FieldContants.BLUE_HUB_POSE,
            ShootingMeasurments.ALL_MEASURMENTS_HUB);
    private final ShootingCalculator deliveryShootingCalculator = new ShootingCalculator(
            FieldContants.BLUE_DELIVERY_POSE, ShootingMeasurments.ALL_MEASURMENTS_DELIVRY);

    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    private final AllCommands allCommands = new AllCommands(fourbar, roller, flyWheel, hood, index, elevator);

    private final PowerDistribution pdh = new PowerDistribution();

    private final NaturalXboxController driverController = new NaturalXboxController(
            RobotMap.Controllers.DRIVER_PORT);
    private final NaturalXboxController operatorController = new NaturalXboxController(
            RobotMap.Controllers.OPERATOR_PORT);

    public RobotContainer() {
        pdh.setSwitchableChannel(true);
        new Trigger(DriverStation::isDisabled).whileTrue(swerveCommands.stop().alongWith(allCommands.stopAll()));
        configureDrive();
        configureOperator();
        configureAuto();
    }

    private void configureDrive() {
        TunableCommand driveCommand = swerveCommands.driverController(
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX,
                driverController.leftBumper().negate()::getAsBoolean,
                driverController.rightBumper()::getAsBoolean);

        swerve.setDefaultCommand(driveCommand);
        TunablesManager.add("Swerve/drive command", driveCommand.fullTunable());

        driverController.x().onTrue(swerveCommands.xWheelLock());

        TunablesManager.add("Swerve/modules control mode",
                swerveCommands.controlModules(
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightY).fullTunable());

        driverController.a().onTrue(new InstantCommand(swerve::resetYawZero));
    }

    public void configureOperator() {
        operatorController.a().whileTrue(allCommands.intake());

        BooleanSupplier isShootingHub = operatorController.b();

        DoubleSupplier hoodAngleSupplier = () -> (isShootingHub.getAsBoolean() ? hubShootingCalculator
                : deliveryShootingCalculator).getHoodAngleDegrees();
        DoubleSupplier flywheelSpeedSupplier = () -> (isShootingHub.getAsBoolean() ? hubShootingCalculator
                : deliveryShootingCalculator).getFlyWheelRPM();

        hood.setDefaultCommand(allCommands.hoodCMDs.moveToAngle(hoodAngleSupplier));
        fourbar.setDefaultCommand(allCommands.fourbarCMDs.getToAngleDegrees(AllCommandsConstants.FOURBAR_MID_ANGLE_DEG));

        operatorController.leftTrigger().whileTrue(allCommands.getReadyToShoot(flywheelSpeedSupplier, hoodAngleSupplier));
        operatorController.rightTrigger().whileTrue(allCommands.shoot(flywheelSpeedSupplier, hoodAngleSupplier));

        TunablesManager.add("Tunable Shoot Command", allCommands.tunableShoot().fullTunable());
    }

    public void configureAuto() {
    }

    public void enterSwerveIntoTest() {
        swerve.costAll();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
