package frc.robot.subsystems.intake.roller;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RollerCommands {
    private Roller roller;

    public RollerCommands(Roller roller){
        this.roller = roller;
    }
    
    public Command takeIn(DoubleSupplier speed){
        return roller.run(() -> roller.setSpeed(speed.getAsDouble()))
        .finallyDo(roller::stop)
        .withName("Take ball in");
    }
    
    public Command stop(){
        return roller.run(roller::stop)
        .withName("Stop");
    }
}
