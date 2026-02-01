package frc.robot.subsystems.intake.roller;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

public class RollerCommands {
    private Roller roller;

    public RollerCommands(Roller roller){
        this.roller = roller;
    }
    
    public Command spin(DoubleSupplier speed){
        return roller.run(() -> roller.setSpeedRPM(speed.getAsDouble()))
        .finallyDo(roller::stop)
        .withName("Take ball in");
    }

    public Command spin (double speed){
        return spin(() -> speed);
    }
    
    public Command manualController(DoubleSupplier speed){
        return roller.run(() -> roller.setSpeedRPM(speed.getAsDouble()*RollerConstants.MAX_RPM))
        .finallyDo(roller::stop)
        .withName("Manual Controller");
    }

    public Command stop(){
        return roller.run(roller::stop)
        .withName("Stop");
    }
}
