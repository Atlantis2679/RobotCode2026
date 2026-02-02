package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.allCommands.AllCommands;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.intake.roller.Roller;
import frc.robot.subsystems.intake.slapdown.Slapdown;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utils.NaturalXboxController;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Slapdown slapdown = new Slapdown();
    private final Roller roller = new Roller();
    private final Index index = new Index();
    private final Hood hood = new Hood();
    private final FlyWheel flyWheel = new FlyWheel();

    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    private final AllCommands allCommands = new AllCommands(slapdown, roller, flyWheel, hood, swerve, index);
    
    private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

    private final NaturalXboxController driverController = new NaturalXboxController(
            RobotMap.Controllers.DRIVER_PORT);
    private final NaturalXboxController operatorController = new NaturalXboxController(
        RobotMap.Controllers.OPERATOR_PORT
    );

    public RobotContainer() {
        new Trigger(DriverStation::isDisabled).whileTrue(swerveCommands.stop());
        configureDrive();
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

        driverController.a().onTrue(new InstantCommand(() -> swerve.resetYaw(0)));
    }

    public void configureOperator() {
        operatorController.a()
            .onTrue(allCommands.startIntake())
            .onFalse(allCommands.stopIntake());
        operatorController.leftTrigger().whileTrue(allCommands.tunableShootPrep());
        operatorController.rightTrigger().whileTrue(allCommands.shoot());

        TunablesManager.add("Shoot Prep Commad", allCommands.tunableShootPrep().fullTunable());

    }

    public void configureAuto(){
        NamedCommands.registerCommand("stopAll", allCommands.stopAll());

        NamedCommands.registerCommand("startIntake", allCommands.startIntake());
        NamedCommands.registerCommand("stopIntake", allCommands.stopIntake());

        NamedCommands.registerCommand("shoot", allCommands.shoot());

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto Chooser", autoChooser);
        Field2d field = new Field2d();

        SmartDashboard.putData(field);

    }

    public void enterSwerveIntoTest() {
        swerve.costAll();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
