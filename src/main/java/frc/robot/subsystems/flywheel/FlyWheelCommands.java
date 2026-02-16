package frc.robot.subsystems.flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class FlyWheelCommands {
    private final FlyWheel flyWheel;

    public FlyWheelCommands(FlyWheel flyWheel) {
        this.flyWheel = flyWheel;
    }

    public Command reachSpeed(DoubleSupplier speedRPM) {
        return flyWheel.runOnce(flyWheel::resetPID)
                .andThen(flyWheel.run(() -> {
                    flyWheel.setVoltage(flyWheel.calculateFeedForward(speedRPM.getAsDouble(), true));
                })).withName("Flywheel reach speed");
    }

    public Command manualController(DoubleSupplier precentageVoltage) {
        return flyWheel.run(() -> 
            flyWheel.setVoltage(precentageVoltage.getAsDouble() * FlyWheelConstants.MAX_VOLTAGE))
            .finallyDo(flyWheel::stop).withName("Flywheel manual controller");
    }
}
