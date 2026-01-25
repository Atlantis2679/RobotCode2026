package frc.robot.subsystems.index;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

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

    public Command insert(DoubleSupplier volt){
        return index.run(() -> index.setIndexerVolt(volt.getAsDouble()))
            .finallyDo(()-> index.setIndexerVolt(0))
            .withName("Rotate Indexer");
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
}
