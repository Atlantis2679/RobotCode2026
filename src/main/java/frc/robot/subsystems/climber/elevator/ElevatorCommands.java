package frc.robot.subsystems.climber.elevator;

import static frc.robot.subsystems.climber.elevator.ElevatorConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.valueholders.ValueHolder;

public class ElevatorCommands {
    private Elevator elevator;

    public ElevatorCommands(Elevator elevator) {
        this.elevator = elevator;
    }

    public Command moveToHeight(DoubleSupplier desiredHeight) {
        ValueHolder<TrapezoidProfile.State> referenceState = new ValueHolder<TrapezoidProfile.State>(null);
        return elevator.runOnce(() -> {
            elevator.resetPID();
            referenceState.set(new TrapezoidProfile.State(elevator.getHeightMeters(), elevator.getHeightVelocity()));
        }).andThen(elevator.run(() -> {
            referenceState.set(elevator.calculateTrapezoidProfile(
                    0.02,
                    referenceState.get(),
                    new TrapezoidProfile.State(desiredHeight.getAsDouble(), 0)));

            double voltage = elevator.calculateFeedForward(
                    referenceState.get().position,
                    referenceState.get().velocity, true);

            elevator.setElevatorVoltage(voltage);
        }));
    }

    public Command moveToHeight(double desiredHeight) {
        return moveToHeight(() -> desiredHeight);
    }

    public Command elevatorManualController(DoubleSupplier elevatorSpeed) {
        return elevator.run(() -> {
            double demandSpeed = elevatorSpeed.getAsDouble();

            double feedForward = elevator.calculateFeedForward(elevator.getHeightMeters(), 0, false);

            elevator.setElevatorVoltage(feedForward + demandSpeed * MAX_VOLTAGE);
        }).withName("elevatorManualController");
    }

    public Command stop() {
        return elevator.run(() -> elevator.stop()).withName("stopElevator");
    }
}