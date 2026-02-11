package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;
import team2679.atlantiskit.valueholders.ValueHolder;

import static frc.robot.subsystems.hood.HoodConstants.*;

public class HoodCommands {
    private final Hood hood;

    public HoodCommands(Hood hood) {
        this.hood = hood;
        TunablesManager.add("TunableSetVoltages/HoodSetVoltage", tunableSetVoltage().fullTunable());
    }

    public Command moveToAngle(DoubleSupplier angle) {
        ValueHolder<TrapezoidProfile.State> referenceState = new ValueHolder<TrapezoidProfile.State>(null);
        return hood.runOnce(() -> {
            hood.resetPID();
            referenceState.set(new TrapezoidProfile.State(hood.getAngleDegrees(), hood.getVelocity()));
        }).andThen(hood.run(() -> {
            referenceState.set(hood.calculateTrapezoidProfile(
                    0.02,
                    referenceState.get(),
                    new TrapezoidProfile.State(angle.getAsDouble(), 0)));

            double volt = hood.calculateFeedForward(
                    referenceState.get().position,
                    referenceState.get().velocity, true);

            hood.setVoltage(volt);
        })).withName("Hood move to angle");
    }

    public Command moveToAngle(double angle) {
        return moveToAngle(() -> angle);
    }

    private TunableCommand tunableSetVoltage() {
        // return hood.run(() -> {
        // hood.setHoodVoltage(volt.getAsDouble());
        // }).finallyDo(hood::stop).withName("Set voltage");
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder voltage = tunablesTable.addNumber("voltage", 0.0);
            return hood.run(() -> hood.setVoltage(voltage.get())).finallyDo(hood::stop)
                    .withName("Tunable hood set voltage");
        });
    }

    public Command manualController(DoubleSupplier speed) {
        return hood.run(() -> {
            double demandSpeed = speed.getAsDouble();
            double feedForward = hood.calculateFeedForward(hood.getAngleDegrees(), 0, false);
            hood.setVoltage(feedForward + demandSpeed * MAX_VOLTAGE);
        }).finallyDo(hood::stop).withName("Hood manual controller");
    }
}
