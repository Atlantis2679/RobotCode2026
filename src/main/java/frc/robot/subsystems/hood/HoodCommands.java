package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.utils.MathUtils;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;

import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodCommands {
    private final Hood hood;

    public HoodCommands(Hood hood) {
        this.hood = hood;
        TunablesManager.add("TunableSetVoltages/HoodSetVoltage", tunableSetVoltage().fullTunable());
        TunablesManager.add("Hood/Cosine Follower", cosineWaveFollower().fullTunable());
        TunablesManager.add("Hood/tunableHoming", tunableHoming().fullTunable());
    }

    public Command moveToAngle(DoubleSupplier angle) {
        return hood.runOnce(() -> {
            hood.resetPID();
        }).andThen(homing()).andThen(hood.run(() -> {
            hood.setVoltage(hood.calculatePID(angle.getAsDouble()));
        })).withName("Hood move to angle");
    }

    public Command moveToAngle(double angle) {
        return moveToAngle(() -> angle);
    }

    private TunableCommand tunableSetVoltage() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder voltage = tunablesTable.addNumber("voltage", 0.0);
            return hood.run(() -> hood.setVoltage(voltage.get())).finallyDo(hood::stop)
                    .withName("Tunable hood set voltage");
        });
    }

    public Command homing() {
        return hood.run(() -> hood.setVoltage(HOMING_VOLTAGE)).onlyWhile(() -> !hood.isCalibrated())
                .finallyDo(hood::stop).withName("Homing");
    }

    public TunableCommand tunableHoming() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder voltage = tunablesTable.addNumber("voltage", HOMING_VOLTAGE);
            return Commands.sequence(
                hood.runOnce(() -> hood.setCurrentLimit(HOMING_CURRENT_LIMIT)),
                hood.runOnce(() -> hood.calibrated = false),
                hood.run(() -> hood.setVoltage(voltage.get()))
                    .onlyWhile(() -> !hood.isCalibrated()),
                hood.runOnce(() -> hood.setCurrentLimit(CURRENT_LIMIT))
            ).finallyDo(hood::stop).withName("Tunable Homing");
        });
    }

    public TunableCommand cosineWaveFollower() {
        return TunableCommand.wrap((tunablesTable) -> {
            MathUtils.CosineWaveFollower cosineWaveFollower = new MathUtils.CosineWaveFollower(hood.minAngle, hood.maxAngle);
            tunablesTable.addChild("Cosine wave follower", cosineWaveFollower);
            return hood.run(() -> {
                double angle = cosineWaveFollower.getNext();
                double voltage = hood.calculatePID(angle);
                hood.setVoltage(voltage);
            });
        });
    }

    public Command manualController(DoubleSupplier speed) {
        return hood.run(() -> {
            hood.setVoltage(speed.getAsDouble() * MAX_VOLTAGE);
        }).finallyDo(hood::stop).withName("Hood manual controller");
    }
}
