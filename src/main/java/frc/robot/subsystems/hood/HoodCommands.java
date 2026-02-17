package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;

import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodCommands {
    private final Hood hood;

    public HoodCommands(Hood hood) {
        this.hood = hood;
        TunablesManager.add("TunableSetVoltages/HoodSetVoltage", tunableSetVoltage().fullTunable());
    }

    public Command moveToAngle(DoubleSupplier angle) {
        return hood.runOnce(() -> {
            hood.resetPID();
        }).andThen(hood.run(() -> {
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

    public TunableCommand tunableHoming() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder voltage = tunablesTable.addNumber("voltage", -1.0);
            return hood.run(() -> hood.setVoltage(voltage.get())).onlyWhile(() -> !hood.isCalibrated()).finallyDo(hood::stop).withName("Homing");
        });
    }

    public Command manualController(DoubleSupplier speed) {
        return hood.run(() -> {
            hood.setVoltage(speed.getAsDouble() * MAX_VOLTAGE);
        }).finallyDo(hood::stop).withName("Hood manual controller");
    }
}
