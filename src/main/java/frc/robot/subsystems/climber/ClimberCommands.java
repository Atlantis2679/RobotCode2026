package frc.robot.subsystems.climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class ClimberCommands {
    private Climber climber;

    public ClimberCommands(Climber climber) {
        this.climber = climber;
    }
    public Command moveToHeight(DoubleSupplier desiredHeight) {
        return null; //temporary(I've done enough for today)
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

    public Command stopElevator(){
        return climber.run(() -> climber.setElevatorVoltage(0)).withName("stopElevator");
    }

    public Command stopPivot() {
        return climber.run(() -> climber.setPivotVoltage(0)).withName("stopPivot");
    }

    public Command stop() {
        return climber.run(climber::stop).withName("stopClimber");
    }
}
