package frc.robot.allCommands;

import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.FlyWheelCommands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodCommands;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexCommands;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerCommands;
import frc.robot.subsystems.forebar.Forebar;
import frc.robot.subsystems.forebar.ForebarCommands;
import frc.robot.subsystems.swerve.Swerve;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;

import static frc.robot.allCommands.AllCommandsConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AllCommands {
    private Forebar forbar;
    private Roller roller;
    private FlyWheel flyWheel;
    private Hood hood;
    private Swerve swerve;
    private Index index;
    private Elevator elevator;

    public ForebarCommands forebarCMDs;
    public RollerCommands rollerCMDs;
    public FlyWheelCommands flyWheelCMDs;
    public HoodCommands hoodCMDs;
    public IndexCommands indexCMDs;
    public ElevatorCommands elevatorCMDs;

    public AllCommands(Forebar forebar, Roller roller, FlyWheel flyWheel, Hood hood, Index index,
            Elevator elevator) {
        this.forbar = forebar;
        this.roller = roller;
        this.flyWheel = flyWheel;
        this.hood = hood;
        this.index = index;
        this.elevator = elevator;

        forebarCMDs = new ForebarCommands(this.forbar);
        rollerCMDs = new RollerCommands(this.roller);
        flyWheelCMDs = new FlyWheelCommands(this.flyWheel);
        hoodCMDs = new HoodCommands(this.hood);
        indexCMDs = new IndexCommands(this.index);
        elevatorCMDs = new ElevatorCommands(this.elevator);
    }

    public Command intake() {
        return Commands.parallel(
                forebarCMDs.getToAngleDegrees(FORBAR_OPEN_ANGLE_DEG),
                rollerCMDs.spin(ROLLER_SPEED_RPM)).withName("intake");
    }

    public Command getReadyToShoot(DoubleSupplier speedRPM, DoubleSupplier angle) {
        return Commands.parallel(
                hoodCMDs.moveToAngle(angle),
                flyWheelCMDs.reachSpeed(speedRPM)).withName("getReadyToShoot");
    }

    public Command shoot(DoubleSupplier speedRPM, DoubleSupplier angle) {
        return Commands.parallel(
                getReadyToShoot(speedRPM, angle),
                Commands.waitUntil(
                        () -> flyWheel.isAtSpeed(speedRPM.getAsDouble()) && hood.isAtAngle(angle.getAsDouble()))
                        .andThen(indexCMDs.spinBoth(INDEXER_VOLTAGE, SPINDEX_VOLTAGE)))
                .withName("shoot");
    }

    public TunableCommand tunableShoot() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder speedHolder = tunablesTable.addNumber("speedRPM", 0.0);
            DoubleHolder hoodAngleHolder = tunablesTable.addNumber("angle", 0.0);
            return shoot(speedHolder::get, hoodAngleHolder::get)
                    .withName("tunableShoot");
        });
    }

    public Command climb() {
        return elevatorCMDs.moveToHeight(ELEVATOR_CLIMB_HEIGHT_METERS).withName("climb");
    }

    public Command unclimb() {
        return elevatorCMDs.moveToHeight(ELEVATOR_UNCLIMB_HEIGHT_METERS).withName("unclimb");
    }

    public Command stopAll() {
        return Commands.run(() -> {
            forbar.stop();
            roller.stop();
            flyWheel.stop();
            hood.stop();
            swerve.stop();
            elevator.stop();
        }, forbar, roller, flyWheel, hood, swerve, elevator)
                .ignoringDisable(true)
                .withName("stopAll");
    }

    public Command manualController(DoubleSupplier flywheelSpeed, DoubleSupplier hoodSpeed,
            DoubleSupplier forbarSpeed, DoubleSupplier rollerSpeed, DoubleSupplier indexSpeed) {
        return Commands.parallel(
                flyWheelCMDs.manualController(flywheelSpeed),
                hoodCMDs.manualController(hoodSpeed),
                forebarCMDs.manualController(forbarSpeed),
                rollerCMDs.manualController(rollerSpeed),
                indexCMDs.manualController(indexSpeed))
                .withName("manualController");
    }

}
