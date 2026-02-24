package frc.robot.subsystems.flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;

public class FlyWheelCommands {
    private final FlyWheel flyWheel;

    public FlyWheelCommands(FlyWheel flyWheel) {
        this.flyWheel = flyWheel;
        TunablesManager.add("TunableSetVoltages/FlywheelSetVoltage", tunableSetVoltage().fullTunable());
    }

    public Command reachSpeed(DoubleSupplier speedRPM) {
        return flyWheel.runOnce(flyWheel::resetPID)
                .andThen(flyWheel.run(() -> {
                    flyWheel.setVoltage(flyWheel.calculateFeedForward(speedRPM.getAsDouble(), true));
                })).finallyDo(flyWheel::stop).withName("Flywheel reach speed");
    }

    public Command manualController(DoubleSupplier precentageVoltage) {
        return flyWheel.run(() -> {
            flyWheel.setVoltage(precentageVoltage.getAsDouble() * FlyWheelConstants.MAX_VOLTAGE);
        }).finallyDo(flyWheel::stop).withName("Flywheel manual controller");
    }

    private TunableCommand tunableSetVoltage() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder voltage = tunablesTable.addNumber("voltage", 0.0);
            return flyWheel.run(() -> flyWheel.setVoltage(voltage.get())).finallyDo(flyWheel::stop)
                    .withName("Tunable Flywheel set voltage");
        });
    }
}
