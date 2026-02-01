package frc.robot.subsystems.index;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.subsystems.index.IndexConstants.*;

public class IndexCommands {
    private Index index;

    public IndexCommands(Index index){
        this.index = index;
    }

    public Command spin(DoubleSupplier volt){
        return index.run(() -> index.setSpindexVolt(volt.getAsDouble()))
            .finallyDo(() -> index.setSpindexVolt(0))
            .withName("Rotate Spindex");
    }

    public Command spin(double volt){
        return spin(() -> volt);
    }

    public Command insert(DoubleSupplier volt){
        return index.run(() -> index.setIndexerVolt(volt.getAsDouble()))
            .finallyDo(()-> index.setIndexerVolt(0))
            .withName("Rotate Indexer");
    }
    
    public Command insert(double volt){
        return insert(() -> volt);
    }

    public Command spinBoth(DoubleSupplier indexerVolt, DoubleSupplier spindexVolt){
        return index.run(() -> {
            index.setIndexerVolt(indexerVolt.getAsDouble());
            index.setSpindexVolt(spindexVolt.getAsDouble());
        }).finallyDo(index::stop)
        .withName("Set voltage for both motors");    
    }

    public Command spinBoth(double indexerVolt, double spindexVolt){
        return spinBoth(() -> indexerVolt, () -> spindexVolt);
    }

    public Command stopSpin(){
        return index.run(() -> index.setSpindexVolt(0));
    }

    public Command stopIn(){
        return index.run(() -> index.setIndexerVolt(0));
    }

    public Command stop() {
        return index.run(index::stop);
    }

    public Command manualController(DoubleSupplier volt){
        return spinBoth(volt.getAsDouble()*MAX_VOLT, volt.getAsDouble()*MAX_VOLT);
    }
}
