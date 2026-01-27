package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.valueholders.ValueHolder;

public class ClimberCommands {
    private Climber climber;

    public ClimberCommands(Climber climber) {
        this.climber = climber;
    }

    public Command moveToHeight(DoubleSupplier desiredHeight) {
        ValueHolder<TrapezoidProfile.State> referenceState = new ValueHolder<TrapezoidProfile.State>(null);
        return climber.runOnce(() -> {
            climber.resetPID();
            referenceState.set(new TrapezoidProfile.State(climber.getHeight(), climber.getHeightVelocity()));
        }).andThen(climber.run(() -> {
            referenceState.set(climber.calculateTrapezoidProfile(
                    0.02,
                    referenceState.get(),
                    new TrapezoidProfile.State(desiredHeight.getAsDouble(), 0)));

            double voltage = climber.calculateFeedForward(
                    referenceState.get().position,
                    referenceState.get().velocity, true);

            climber.setElevatorVoltage(voltage);
        }));
    }

    public Command moveToHeight(double desiredHeight) {
        return moveToHeight(() -> desiredHeight);
    }

    public Command elevatorManualController(DoubleSupplier elevatorSpeed) {
        return climber.run(() -> {
            double demandSpeed = elevatorSpeed.getAsDouble();

            double feedForward = climber.calculateFeedForward(climber.getHeight(), 0, false);

            climber.setElevatorVoltage(feedForward + demandSpeed * ClimberConstants.Elevator.MAX_VOLTAGE);
        }).withName("elevatorManualController");
    }

    public Command pivotManualController(DoubleSupplier pivotSpeed) {
        return climber.run(() -> climber.setPivotVoltage(0)).withName("pivotManualController");
    }

    public Command stopElevator() {
        return climber.run(() -> climber.setElevatorVoltage(0)).withName("stopElevator");
    }

    public Command stopPivot() {
        return climber.run(() -> climber.setPivotVoltage(0)).withName("stopPivot");
    }

    public Command stop() {
        return climber.run(climber::stop).withName("stopClimber");
    }
}
