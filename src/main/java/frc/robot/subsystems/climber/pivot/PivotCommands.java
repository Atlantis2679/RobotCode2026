package frc.robot.subsystems.climber.pivot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.valueholders.ValueHolder;

public class PivotCommands {
    private Pivot pivot;

    public PivotCommands(Pivot pivot) {
        this.pivot = pivot;
    }

    public Command moveToAngleDegrees(DoubleSupplier desiredAngle) {
        ValueHolder<TrapezoidProfile.State> referenceState = new ValueHolder<TrapezoidProfile.State>(null);
            return pivot.runOnce(() -> {
                pivot.resetPID();
                referenceState.set(new TrapezoidProfile.State(pivot.getAngleDegrees(), pivot.getAngularVelocity()));
            }).andThen(pivot.run(() -> {
                referenceState.set(pivot.calculateTrapezoidProfile(
                        0.02,
                        referenceState.get(),
                        new TrapezoidProfile.State(desiredAngle.getAsDouble(), 0)));

                double voltage = pivot.calculateFeedForward(
                        referenceState.get().position,
                        referenceState.get().velocity, true);

                pivot.setPivotVoltage(voltage);
            }));
    }
    public Command moveToAngleDegrees(double desiredAngle) {
        return moveToAngleDegrees(() -> desiredAngle);
    }
}
