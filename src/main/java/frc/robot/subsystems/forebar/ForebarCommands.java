package frc.robot.subsystems.forebar;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.valueholders.ValueHolder;

public class ForebarCommands {
    private Forebar forebar;

    public ForebarCommands(Forebar slapdown) {
        this.forebar = slapdown;
    }

    public Command moveToAngle(DoubleSupplier angle) {
        ValueHolder<TrapezoidProfile.State> state = new ValueHolder<TrapezoidProfile.State>(null);
        return forebar.runOnce(() -> {
            state.set(new TrapezoidProfile.State(forebar.getAngleDegrees(), forebar.getVelocity()));
            forebar.resetPID();
        }).andThen(forebar.run(() -> {
            state.set(forebar.calculateTrapezoidProfile(
                    0.02, state.get(), new TrapezoidProfile.State(angle.getAsDouble(), 0)));
            double voltage = forebar.calculateFeedforward(
                    state.get().position, state.get().velocity, true);
            forebar.setVoltage(voltage);
        })).withName("Forbar move to angle");
    }

    public Command getToAngleDegrees(double angle) {
        return moveToAngle(() -> angle);
    }

    public Command manualController(DoubleSupplier speed) {
        return forebar.run(() -> {
            double ignore_mg = forebar.calculateFeedforward(forebar.getAngleDegrees(), forebar.getVelocity(), false);
            forebar.setVoltage(ignore_mg + speed.getAsDouble() * ForebarConstants.MAX_VOLTAGE);
        }).withName("Forbar manual controller");
    }
}
