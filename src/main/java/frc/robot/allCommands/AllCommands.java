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
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;

import static frc.robot.allCommands.AllCommandsConstants.*;

import java.util.function.DoubleSupplier;

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

    public double current_speed_test;
    public double current_angle_test;

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

    public Command shoot(DoubleSupplier speed, DoubleSupplier angle){
        return Commands.waitUntil(() -> 
            flyWheel.isAtSpeed(speed.getAsDouble())&&hood.isAtAngle(angle.getAsDouble()))
            .andThen(indexCMDs.spinBoth(() -> INDEXER_VOLTAGE, () -> SPINDEX_VOLTAGE));
    }

    public Command shoot(){
        return shoot(() -> current_speed_test, () -> current_angle_test);
    }

    public Command shootPrep(DoubleSupplier speedRPM, DoubleSupplier angle){
        return Commands.parallel(
            hoodCMDs.moveToAngle(angle),
            flyWheelCMDs.setSpeed(speedRPM)
        );
    }

    public TunableCommand tunableShootPrep(){
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder speedHolder = tunablesTable.addNumber("speed", FLYWHEEL_SCORING_SPEED_RPM);
            DoubleHolder angleHolder = tunablesTable.addNumber("angle", HOOD_SCORING_ANGLE);

            current_angle_test = angleHolder.get();
            current_speed_test = speedHolder.get();

            return shootPrep(() -> speedHolder.get(), () -> angleHolder.get())
            .withName("Delivery tunable command");
        });
    }

    public Command startIntake(){
        return Commands.parallel(
            slapdownCMDs.goToAngleDeg(SLAPDOWN_OPEN_ANGLE_DEG),
            rollerCMDs.spin(ROLLER_SPEED_RPM)
        );
    }

    public Command stopIntake(){
        return Commands.parallel(
            slapdownCMDs.goToAngleDeg(SLAPDOWN_MID_ANGLE_DEG),
            Commands.waitUntil(() -> (slapdown.isAtAngle(SLAPDOWN_MID_ANGLE_DEG)))
                .andThen(rollerCMDs.stop()));
    }

    public Command stopAll(){
        return Commands.run(() -> {
            slapdownCMDs.stop();
            rollerCMDs.stop();
            flyWheelCMDs.stop();
            hoodCMDs.stop();
            swerveCMDs.stop();
        }, slapdown, roller, flyWheel, hood, swerve)
        .ignoringDisable(true)
        .withName("Stop All");
    }

    public Command manualController(DoubleSupplier flywheelSpeed, DoubleSupplier hoodSpeed, 
        DoubleSupplier slapdwonSpeed, DoubleSupplier rollerSpeed, DoubleSupplier indexVolt){
            return Commands.parallel(
                flyWheelCMDs.manualController(flywheelSpeed),
                hoodCMDs.manualController(hoodSpeed),
                slapdownCMDs.manualController(slapdwonSpeed),
                rollerCMDs.manualController(rollerSpeed),
                indexCMDs.manualController(indexVolt)
            );
    }

}
