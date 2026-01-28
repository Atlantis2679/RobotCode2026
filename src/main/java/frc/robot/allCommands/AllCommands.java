package frc.robot.allCommands;

import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.FlyWheelCommands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodCommands;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexCommands;
import frc.robot.subsystems.intake.roller.Roller;
import frc.robot.subsystems.intake.roller.RollerCommands;
import frc.robot.subsystems.intake.slapdown.Slapdown;
import frc.robot.subsystems.intake.slapdown.SlapdownCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AllCommands {
    private Slapdown slapdown;
    private Roller roller;
    private FlyWheel flyWheel;
    private Hood hood;
    private Swerve swerve;
    private Index index;

    private SlapdownCommands slapdownCMDs;
    private RollerCommands rollerCMDs;
    private FlyWheelCommands flyWheelCMDs;
    private HoodCommands hoodCMDs;
    private SwerveCommands swerveCMDs;
    private IndexCommands indexCMDs;

    public AllCommands(Slapdown slapdown, Roller roller, FlyWheel flyWheel, Hood hood, Swerve swerve, Index index) {
        this.slapdown = slapdown;
        this.roller = roller;
        this.flyWheel = flyWheel;
        this.hood = hood;
        this.swerve = swerve;
        this.index = index;

        slapdownCMDs = new SlapdownCommands(this.slapdown);
        rollerCMDs = new RollerCommands(this.roller);
        flyWheelCMDs = new FlyWheelCommands(this.flyWheel);
        hoodCMDs = new HoodCommands(this.hood);
        swerveCMDs = new SwerveCommands(this.swerve);
        indexCMDs = new IndexCommands(this.index);
    }

    public Command startIntake() {
        return slapdownCMDs.goToAngleDeg(() -> AllCommandsConstants.SLAPDOWN_OPEN_ANGLE_DEG)
                .andThen(() -> rollerCMDs.spin(() -> 1));
    }

    public Command stopIntake() {
        return rollerCMDs.stop()
                .andThen(() -> slapdownCMDs.goToAngleDeg(() -> AllCommandsConstants.SLAPDOWN_MID_ANGLE_DEG));
    }

    public Command startDelivering() { // Currently assumes orientation is correct
        return Commands.runEnd(
                () -> Commands.parallel(
                        indexCMDs.spin(() -> AllCommandsConstants.INDEX_VOLT),
                        flyWheelCMDs.setSpeed(() -> AllCommandsConstants.FLYWHEEL_DELIVERY_SPEED),
                        hoodCMDs.moveToAngle(() -> AllCommandsConstants.HOOD_DELIVERY_ANGLE))
                        .until(() -> flyWheel.isAtSpeed(AllCommandsConstants.FLYWHEEL_DELIVERY_SPEED)
                                && hood.isAtAngle(AllCommandsConstants.HOOD_DELIVERY_ANGLE)),
                () -> indexCMDs.insert(() -> AllCommandsConstants.INDEX_VOLT),
                index, flyWheel, hood);
    }

    public Command stopDelivery(){
        return Commands.parallel(
            indexCMDs.stop(),
            flyWheelCMDs.stop());
    }
}