package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.allCommands.AllCommands;
import frc.robot.shooting.ShotControl;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.fourbar.Fourbar;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.poseestimation.PoseEstimator;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.PathPlanner;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utils.CommandsUtils;
import frc.robot.utils.NaturalXboxController;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Fourbar fourbar = new Fourbar();
    private final Roller roller = new Roller();
    private final Index index = new Index();
    private final Hood hood = new Hood();
    private final FlyWheel flyWheel = new FlyWheel();
    private final Elevator elevator = new Elevator();
    private final Vision vision = new Vision();

    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    private final AllCommands allCommands = new AllCommands(fourbar, roller, flyWheel, hood, index, elevator);

    private final PowerDistribution pdh = new PowerDistribution();

    private final NaturalXboxController driverController = new NaturalXboxController(
            RobotMap.Controllers.DRIVER_PORT);
    private final NaturalXboxController operatorController = new NaturalXboxController(
            RobotMap.Controllers.OPERATOR_PORT);

    private static final LoggedDashboardChooser<Boolean> isRedAlliance = new LoggedDashboardChooser<>("alliance");

    static {
        isRedAlliance.addDefaultOption("red", true);
        isRedAlliance.addOption("blue", false);
        SmartDashboard.putBoolean("isRedAlliance", RobotContainer.isRedAlliance());
    }

    private final ShotControl shotControl = new ShotControl();

    private final Command shotCommand = CommandsUtils.dynamicSwitchBetweenCommands(
            shotControl::shoot,
            allCommands.getReadyToShoot(shotControl::getRpm, shotControl::getAngle),
            allCommands.shoot(shotControl::getRpm, shotControl::getAngle));

    private final Trigger shotTrigger = operatorController.leftTrigger();

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
        new Trigger(DriverStation::isDisabled).whileTrue(swerveCommands.stop().alongWith(allCommands.stopAll()));
        configureDrive();
        configureOperator();
        configureAuto();
    }

    private void configureDrive() {
        TunableCommand driveCommand = swerveCommands.driverController(
                driverController::getLeftY,
                driverController::getLeftX,
                () -> {
                    if (shotTrigger.getAsBoolean()) {
                        return shotControl.getDriveRotationRPS() / SwerveConstants.DriverController.DRIVER_MAX_ANGULAR_VELOCITY_RPS;
                    } else {
                        return driverController.getRightX();
                    }
                },
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

        driverController.a().onTrue(new InstantCommand(() -> {
            swerve.resetGyroYawZero();
            PoseEstimator.getInstance().resetYawZero();
        }));
    }

    public void configureOperator() {
        TunablesManager.add("Shot Control", shotControl);

        operatorController.a().whileTrue(allCommands.intake());

        hood.setDefaultCommand(allCommands.hoodDefaultMove(shotControl::getAngle));
        fourbar.setDefaultCommand(allCommands.fourbarMoveToRest());
        
        shotTrigger.whileTrue(shotCommand);
        TunablesManager.add("Tunable Shoot Command", allCommands.tunableShoot().fullTunable());
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
        NamedCommands.registerCommand("shoot", shotCommand);

        autoChooser = AutoBuilder.buildAutoChooser();
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
        swerve.costAll();
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
        return autoChooser.getSelected();
    }
}