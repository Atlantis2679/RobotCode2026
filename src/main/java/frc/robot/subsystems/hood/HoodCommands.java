package frc.robot.subsystems.hood;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.valueholders.ValueHolder;

public class HoodCommands {
    private final Hood hood;

    public HoodCommands(Hood hood){
        this.hood = hood;
    }

    public Command moveToAngle(DoubleSupplier angle){
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
                referenceState.get().velocity);

            hood.setHoodVoltage(volt);
        }));
    }

    public Command moveToAngle(double angle) {
        return moveToAngle(() -> angle);
    }

    public Command manualController(DoubleSupplier speed){
        return hood.run(() -> {
            hood.setHoodVoltage(speed.getAsDouble() * 8);
        });
    }
}
