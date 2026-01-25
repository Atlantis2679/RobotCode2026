package frc.robot.subsystems.flywheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.valueholders.ValueHolder;

public class FlyWheelCommands {
    private final FlyWheel flyWheel;


    public FlyWheelCommands(FlyWheel flyWheel){
        this.flyWheel = flyWheel;
    }


    public Command setRPM(DoubleSupplier RPM){
        ValueHolder<TrapezoidProfile.State> referenceState = new ValueHolder<TrapezoidProfile.State>(null);

        return flyWheel.runOnce(() -> {
            flyWheel.resetPID();
            referenceState.set(new TrapezoidProfile.State(flyWheel.getVelocity(), 0));
            
        }).andThen(flyWheel.run(() -> {
            referenceState.set(flyWheel.calculateTrapezoidProfile(
                0.02,
                referenceState.get(),
                new TrapezoidProfile.State(RPM.getAsDouble(), 0)
            ));

            double voltage = flyWheel.CalcVolts(
                referenceState.get().position,
                referenceState.get().velocity
            );
            
            flyWheel.setVoltage(voltage);
        })).withName("FlyWheel get to speed");
    }

    public Command manualController(DoubleSupplier flyWheelSpeed){
        return flyWheel.run(() ->  {

            flyWheel.setVoltage(flyWheelSpeed.getAsDouble() * FlyWheelConstants.MAX_VOLTAGE);

        }).withName("FlyWheel manual controller");
    }
}
