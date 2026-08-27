package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.allCommands.AllCommands;
import frc.robot.shooting.ShotControl;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.fourbar.Fourbar;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.poseestimation.PoseEstimator;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveConstants.PathPlanner;
import frc.robot.utils.CommandsUtils;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.NaturalXboxController;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.BooleanHolder;
import team2679.atlantiskit.valueholders.DoubleHolder;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Fourbar fourbar = new Fourbar();
    private final Roller roller = new Roller();
    private final Index index = new Index();
    private final Hood hood = new Hood();
    private final FlyWheel flyWheel = new FlyWheel();
    private final Vision vision = new Vision();

    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    private final AllCommands allCommands = new AllCommands(fourbar, roller, flyWheel, hood, index);

    private final PowerDistribution pdh = new PowerDistribution();
    private final NaturalXboxController driverController = new NaturalXboxController( RobotMap.Controllers.DRIVER_PORT); private final NaturalXboxController operatorController = new NaturalXboxController(
            RobotMap.Controllers.OPERATOR_PORT);

    private static final LoggedDashboardChooser<Boolean> isRedAlliance = new LoggedDashboardChooser<>("alliance");

    private static final BooleanHolder isAutoTurn = new BooleanHolder(false);

    static {
        isRedAlliance.addDefaultOption("red", true);
        isRedAlliance.addOption("blue", false);
        SmartDashboard.putBoolean("isRedAlliance", RobotContainer.isRedAlliance());
    }

    private final ShotControl shotControl = new ShotControl();

    private SendableChooser<Command> autoChooser = null;

    public RobotContainer() {
        pdh.setSwitchableChannel(true);
        isRedAlliance.onChange((isRedAlliance) -> {
            swerve.resetGyroYawZero();
            PoseEstimator.getInstance().resetYawZero();
        });
        TunablesManager.add("Reset Yaw", new Tunable() {
            @Override
            public void initTunable(TunableBuilder builder) {
                DoubleHolder angleToReset = new DoubleHolder(0);
                builder.addDoubleProperty("angleToResetDegrees", angleToReset::get, angleToReset::set);
                builder.addChild("Reset!", new InstantCommand(() -> {
                    swerve.resetGyroYaw(angleToReset.get());
                    PoseEstimator.getInstance().resetYaw(Rotation2d.fromDegrees(angleToReset.get()));
                }));
            }
        });
        TunablesManager.add("Reset Pose", new InstantCommand(() -> {
            PoseEstimator.getInstance().resetPose(new Pose2d());
            swerve.resetGyroYawZero();
        }));
        TunablesManager.add("PoseEstimator", PoseEstimator.getInstance());
        new Trigger(DriverStation::isDisabled).whileTrue(swerveCommands.stop().alongWith(allCommands.stopAll()));
        SignalLogger.setPath("/media/sda1/"); // Cofigure pheonixLib logging path
        configureDrive();
        configureOperator();
        configureAuto();
    }

    private void configureDrive() {
        TunableCommand driveCommand = swerveCommands.driverController(
                driverController::getLeftY,
                driverController::getLeftX,
                driverController::getRightX,
                shotControl::getDriveAngleDegrees,
                driverController.leftTrigger().or(isAutoTurn::get),
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

        driverController.start().onTrue(new InstantCommand(() -> {
            swerve.resetGyroYawZero();
            PoseEstimator.getInstance().resetYawZero();
        }));
    }

    public void configureOperator() {
        TunablesManager.add("Shot Control", shotControl);

        operatorController.x().whileTrue(allCommands.intake());
        operatorController.a().whileTrue(allCommands.delivery());

        hood.setDefaultCommand(allCommands.hoodFollow(shotControl::getAngle));

        Command shotCommand = CommandsUtils.dynamicSwitchBetweenCommands(
            shotControl::shoot,
            allCommands.shoot(shotControl::getRpm, shotControl::getAngle),
            allCommands.getReadyToShoot(shotControl::getRpm, shotControl::getAngle));
        
        operatorController.rightTrigger().whileTrue(shotCommand);

        operatorController.leftBumper().whileTrue(allCommands.manualController(
            () -> operatorController.getRightTriggerAxis() * (operatorController.rightBumper().getAsBoolean() ? -1 : 1),
            () -> {
                if (operatorController.y().getAsBoolean()) return 0.2;
                if (operatorController.b().getAsBoolean()) return -0.2;
                return 0;
            },
            operatorController::getRightY,
            operatorController::getRightX,
            operatorController::getLeftY));

        operatorController.povLeft().whileTrue(allCommands.fourbarClose());
        operatorController.povRight().whileTrue(allCommands.fourbarOpen());
        operatorController.povUp().onTrue(new InstantCommand(() -> shotControl.adjustRpmOffset(25)).ignoringDisable(true));
        operatorController.povDown().onTrue(new InstantCommand(() -> shotControl.adjustRpmOffset(-25)).ignoringDisable(true));
        operatorController.y().and(operatorController.leftBumper().negate()).onTrue(new InstantCommand(() -> shotControl.adjustHoodOffset(2.5)).ignoringDisable(true));
        operatorController.b().and(operatorController.leftBumper().negate()).onTrue(new InstantCommand(() -> shotControl.adjustHoodOffset(-2.5)).ignoringDisable(true));

        operatorController.axisGreaterThan(Axis.kRightY.value, 0.1).whileTrue(allCommands.manualFourbar(() -> -operatorController.getRightY()));
        operatorController.axisLessThan(Axis.kRightY.value, -0.1).whileTrue(allCommands.manualFourbar(() -> -operatorController.getRightY()));
        operatorController.axisGreaterThan(Axis.kRightX.value, 0.1).whileTrue(allCommands.manualRoller(operatorController::getRightX));
        operatorController.axisLessThan(Axis.kRightX.value, -0.1).whileTrue(allCommands.manualRoller(operatorController::getRightX));
        operatorController.axisGreaterThan(Axis.kLeftY.value, 0.1).whileTrue(allCommands.manualIndex(() -> -operatorController.getLeftY()));
        operatorController.axisLessThan(Axis.kLeftY.value, -0.1).whileTrue(allCommands.manualIndex(() -> -operatorController.getLeftY()));

        operatorController.leftStick().onTrue(allCommands.rehomeHood());

        TunablesManager.add("Tunable Shoot Command", allCommands.tunableShoot().fullTunable());
        TunablesManager.add("Tunable Shoot With Passing", allCommands.tunableShootWithPassing().fullTunable());
    }

    public void configureAuto() {
        Field2d field = new Field2d();

        PoseEstimator.registerCallbackOnPoseUpdate((pose) -> {
            field.setRobotPose(pose);
        });

        SmartDashboard.putData(field);

        PathPlanner.ROBOT_CONFIG.hasValidConfig(); // Check that configs match GUI

        AutoBuilder.configure(PoseEstimator.getInstance()::getEstimatedPose, PoseEstimator.getInstance()::resetPose,
                swerve::getRobotRelativeChassisSpeeds, (speeds, feedforwards) -> {
                    swerve.driveChassisSpeeds(speeds, true);
                }, PathPlanner.FOLLOWING_CONTROLLER, PathPlanner.ROBOT_CONFIG, RobotContainer::isRedAlliance);
        
        NamedCommands.registerCommand("stopAll", allCommands.stopAll());
        NamedCommands.registerCommand("startIntake", allCommands.intake());
        NamedCommands.registerCommand("stopIntake", allCommands.stopIntake());
        NamedCommands.registerCommand("shoot", allCommands.shoot(shotControl::getRpm, shotControl::getAngle).alongWith(new InstantCommand(() -> isAutoTurn.set(true))).finallyDo(() -> isAutoTurn.set(false)));

        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.addOption("Just shoot mid", allCommands.shoot(shotControl::getRpm, shotControl::getAngle));
        SmartDashboard.putData("Auto Chooser", autoChooser);

        autoChooser.onChange((command) -> {
            if (!command.getName().equals("None")) {
                try {
                    List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(command.getName());
                    List<Pose2d> poses = new ArrayList<>();

                    for (PathPlannerPath path : paths) {
                        for (Pose2d pose : path.getPathPoses()) {
                            poses.add(FlippingUtil.flipFieldPose(pose));
                        }
                    }
                    field.getObject("Auto Trajectory").setPoses(poses);
                } catch (Exception e) {
                    System.out.println("Auto Trajectory Loading Failed!");
                }
            } else {
                field.getObject("Auto Trajectory").setPose(PoseEstimator.getInstance().getEstimatedPose());
            }
        });
    }

    public void enterSwerveIntoTest() {
        swerve.coastAll();
    }

    public void periodicUpdate() {
        vision.update();
        shotControl.update(
            PoseEstimator.getInstance().getEstimatedPose(),
            isRedAlliance(),
            swerve.getFieldRelativeChassisSpeeds(),
            swerve.getRobotRelativeChassisSpeeds(),
            swerve.getRobotPitchDeg(),
            swerve.getRobotRollDeg()
        );
    }

    public static boolean isRedAlliance() {
        return isRedAlliance.get() != null && isRedAlliance.get();
    }

    public Command getAutonomousCommand() {
        // Command shotCommand = CommandsUtils.dynamicSwitchBetweenCommands(
        //     shotControl::shoot,
        //     allCommands.shoot(shotControl::getRpm, shotControl::getAngle),
        //     allCommands.getReadyToShoot(shotControl::getRpm, shotControl::getAngle));
        // return swerveCommands.autoDrive().andThen(shotCommand);
        return autoChooser.getSelected();
    }
}
 