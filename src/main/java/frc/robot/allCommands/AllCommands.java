package frc.robot.allCommands;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
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
    private Elevator elevator;

    private SlapdownCommands slapdownCMDs;
    private RollerCommands rollerCMDs;
    private FlyWheelCommands flyWheelCMDs;
    private HoodCommands hoodCMDs;
    private SwerveCommands swerveCMDs;
    private IndexCommands indexCMDs;
    private ElevatorCommands elevatorCMDs;

    public AllCommands(Slapdown slapdown, Roller roller, FlyWheel flyWheel, Hood hood, Swerve swerve, Index index, Elevator elevator) {
        this.slapdown = slapdown;
        this.roller = roller;
        this.flyWheel = flyWheel;
        this.hood = hood;
        this.swerve = swerve;
        this.index = index;
        this.elevator = elevator;

        slapdownCMDs = new SlapdownCommands(this.slapdown);
        rollerCMDs = new RollerCommands(this.roller);
        flyWheelCMDs = new FlyWheelCommands(this.flyWheel);
        hoodCMDs = new HoodCommands(this.hood);
        swerveCMDs = new SwerveCommands(this.swerve);
        indexCMDs = new IndexCommands(this.index);
        elevatorCMDs = new ElevatorCommands(this.elevator);
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
    
    public Command getReadyToShoot(double speedRPM, double angle){
        return Commands.parallel(
            hoodCMDs.moveToAngle(angle),
            flyWheelCMDs.setSpeed(speedRPM)
        );
    }

    public TunableCommand tunableGetReadyToShoot(){
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder speedHolder = tunablesTable.addNumber("speed", FLYWHEEL_SPEED_RPM);
            DoubleHolder hoodAngleHolder = tunablesTable.addNumber("angle", HOOD_ANGLE);
            return getReadyToShoot(speedHolder.get(), hoodAngleHolder.get())
            .withName("Get Ready To Shoot Tunable");
        });
    }

    public Command getReadyAndShoot(double speedRPM, double angle){
        return Commands.parallel(
            Commands.none().until(() -> 
                    flyWheel.isAtSpeed(speedRPM)&&hood.isAtAngle(angle))
                .andThen(indexCMDs.spinBoth(INDEXER_VOLTAGE, SPINDEX_VOLTAGE)),

            hoodCMDs.moveToAngle(angle),
            flyWheelCMDs.setSpeed(speedRPM));
    }

    public TunableCommand tunableGetReadyAndShoot(){
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder speedHolder = tunablesTable.addNumber("Flywheel Speed RPM", FLYWHEEL_SPEED_RPM);
            DoubleHolder hoodAngleHolder = tunablesTable.addNumber("Hood Angle", HOOD_ANGLE);
            return getReadyAndShoot(speedHolder.get(), hoodAngleHolder.get())
            .withName("Tunable Get Ready And Shoot");
        });
    }

    public Command stopAll(){
        return Commands.run(() -> {
            slapdown.stop();
            roller.stop();
            flyWheel.stop();
            hood.stop();
            swerve.stop();
            elevator.stop();
        }, slapdown, roller, flyWheel, hood, swerve, elevator)
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
