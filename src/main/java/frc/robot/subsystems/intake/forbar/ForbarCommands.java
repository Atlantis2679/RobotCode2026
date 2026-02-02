package frc.robot.subsystems.intake.forbar;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.valueholders.ValueHolder;

public class ForbarCommands {
    private Forbar slapdown;

    public ForbarCommands(Forbar slapdown) {
        this.slapdown = slapdown;
    }

    public Command getToAngleDegrees(DoubleSupplier angle) {
        ValueHolder<TrapezoidProfile.State> state = new ValueHolder<TrapezoidProfile.State>(null);
        return slapdown.runOnce(() -> {
            state.set(new TrapezoidProfile.State(slapdown.getAngleDegrees(), slapdown.getVelocity()));
            slapdown.resetPID();
        }).andThen(slapdown.run(() -> {
            state.set(slapdown.calculateTrapezoidProfile(
                    0.02, state.get(), new TrapezoidProfile.State(angle.getAsDouble(), 0)));
            double voltage = slapdown.calculateFeedforward(
                    state.get().position, state.get().velocity, true);
            slapdown.setVoltage(voltage);
        }));
    }

    public Command getToAngleDegrees(double angle) {
        return getToAngleDegrees(() -> angle);
    }

    public Command manualController(DoubleSupplier speed) {
        return slapdown.run(() -> {
            double ignore_mg = slapdown.calculateFeedforward(slapdown.getAngleDegrees(), slapdown.getVelocity(), false);
            slapdown.setVoltage(ignore_mg + speed.getAsDouble() * ForbarConstants.MAX_VOLTAGE);
        });
    }

    public Command stop() {
        return slapdown.run(slapdown::stop);
    }
}
