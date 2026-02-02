package frc.robot.subsystems.intake.forbar;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.valueholders.ValueHolder;

public class ForbarCommands {
    private Forbar forbar;

    public ForbarCommands(Forbar slapdown) {
        this.forbar = slapdown;
    }

    public Command moveToAngle(DoubleSupplier angle) {
        ValueHolder<TrapezoidProfile.State> state = new ValueHolder<TrapezoidProfile.State>(null);
        return forbar.runOnce(() -> {
            state.set(new TrapezoidProfile.State(forbar.getAngleDegrees(), forbar.getVelocity()));
            forbar.resetPID();
        }).andThen(forbar.run(() -> {
            state.set(forbar.calculateTrapezoidProfile(
                    0.02, state.get(), new TrapezoidProfile.State(angle.getAsDouble(), 0)));
            double voltage = forbar.calculateFeedforward(
                    state.get().position, state.get().velocity, true);
            forbar.setVoltage(voltage);
        })).withName("Forbar move to angle");
    }

    public Command getToAngleDegrees(double angle) {
        return moveToAngle(() -> angle);
    }

    public Command manualController(DoubleSupplier speed) {
        return forbar.run(() -> {
            double ignore_mg = forbar.calculateFeedforward(forbar.getAngleDegrees(), forbar.getVelocity(), false);
            forbar.setVoltage(ignore_mg + speed.getAsDouble() * ForbarConstants.MAX_VOLTAGE);
        }).withName("Forbar manual controller");
    }
}
