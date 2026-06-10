package frc.robot.allCommands;

import frc.robot.shooting.ShootingCalculator;
// import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.subsystems.elevator.ElevatorCommands;
import frc.robot.subsystems.flywheel.FlyWheel;
import frc.robot.subsystems.flywheel.FlyWheelCommands;
import frc.robot.subsystems.fourbar.Fourbar;
import frc.robot.subsystems.fourbar.FourbarCommands;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodCommands;
import frc.robot.subsystems.index.Index;
import frc.robot.subsystems.index.IndexCommands;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerCommands;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;

import static frc.robot.allCommands.AllCommandsConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class AllCommands {
    private Fourbar fourbar;
    private Roller roller;
    private FlyWheel flyWheel;
    private Hood hood;
    private Index index;

    private FourbarCommands fourbarCMDs;
    private RollerCommands rollerCMDs;
    private FlyWheelCommands flyWheelCMDs;
    private HoodCommands hoodCMDs;
    private IndexCommands indexCMDs;

    public AllCommands(Fourbar fourbar, Roller roller, FlyWheel flyWheel, Hood hood, Index index) {
        this.fourbar = fourbar;
        this.roller = roller;
        this.flyWheel = flyWheel;
        this.hood = hood;
        this.index = index;

        fourbarCMDs = new FourbarCommands(this.fourbar);
        rollerCMDs = new RollerCommands(this.roller);
        flyWheelCMDs = new FlyWheelCommands(this.flyWheel);
        hoodCMDs = new HoodCommands(this.hood);
        indexCMDs = new IndexCommands(this.index);
    }

    public Command intake() {
        return rollerCMDs.spin(ROLLER_VOLTAGE).withName("intake");
    }

    public Command stopIntake() {
        return fourbar.run(fourbar::stop).alongWith(roller.run(roller::stop));
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
                        .andThen(spinIndex()))
                .withName("shoot");
    }

    public Command spinIndex() {
        return indexCMDs.spinBoth(INDEXER_VOLTAGE, SPINDEX_VOLTAGE).withName("SpinIndex");
    }

    public TunableCommand tunableShoot() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder speedHolder = tunablesTable.addNumber("speedRPM", 0.0);
            DoubleHolder hoodAngleHolder = tunablesTable.addNumber("angle", 0.0);
            return getReadyToShoot(speedHolder::get, hoodAngleHolder::get)
                    .withName("tunableShoot");
        });
    }

    public TunableCommand tunableShootWithPassing() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder speedHolder = tunablesTable.addNumber("speedRPM", 0.0);
            DoubleHolder hoodAngleHolder = tunablesTable.addNumber("angle", 0.0);
            DoubleHolder indexVoltage = tunablesTable.addNumber("indexVoltage", INDEXER_VOLTAGE);
            DoubleHolder spindexVoltage = tunablesTable.addNumber("spindexVoltage", SPINDEX_VOLTAGE);
            return getReadyToShoot(speedHolder::get, hoodAngleHolder::get).alongWith(indexCMDs.spinBoth(indexVoltage::get, spindexVoltage::get))
                    .withName("tunableShoot");
        });
    }

    public TunableCommand tunableShootWithDistance(ShootingCalculator shootingCalculator) {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder indexVoltage = tunablesTable.addNumber("indexVoltage", INDEXER_VOLTAGE);
            DoubleHolder spindexVoltage = tunablesTable.addNumber("spindexVoltage", SPINDEX_VOLTAGE);
            DoubleHolder distance = tunablesTable.addNumber("distanceFromTargetMeters", 0.0);
            return Commands.parallel(
                Commands.run(() -> shootingCalculator.update_with_distance(distance.get())),
                getReadyToShoot(shootingCalculator::getFlyWheelRPM, shootingCalculator::getHoodAngleDegrees),
                indexCMDs.spinBoth(indexVoltage::get, spindexVoltage::get)
            ).withName("tunableShootWithDistance");
        });
    }

    public Command hoodFollow(DoubleSupplier angle) {
        return hoodCMDs.moveToAngle(angle);
    }

    public Command fourbarMoveToRest() {
        return fourbarCMDs.moveToAngle(() -> FOURBAR_MID_ANGLE).withName("fourbarMoveToRest");
    }

    public Command stopFlywheel() {
        return flyWheel.run(flyWheel::stop).withName("stopFlywheel");
    }

    public Command stopIndex() {
        return index.run(index::stop).withName("stopIndex");
    }

    public Command stopAll() {
        return Commands.run(() -> {
            fourbar.stop();
            roller.stop();
            flyWheel.stop();
            hood.stop();
        }, fourbar, roller, flyWheel, hood)
                .ignoringDisable(true)
                .withName("stopAll");
    }

    public Command manualController(DoubleSupplier flywheelSpeed, DoubleSupplier hoodSpeed,
            DoubleSupplier fourbarSpeed, DoubleSupplier rollerSpeed, DoubleSupplier indexSpeed) {
        return Commands.parallel(
                flyWheelCMDs.manualController(flywheelSpeed),
                hoodCMDs.manualController(hoodSpeed),
                fourbarCMDs.manualController(fourbarSpeed),
                rollerCMDs.manualController(rollerSpeed),
                indexCMDs.manualController(indexSpeed))
                .withName("manualController");
    }
}
