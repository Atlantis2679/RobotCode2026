package frc.robot.subsystems.index;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.subsystems.index.IndexConstants.*;

public class IndexCommands {
    private Index index;

    public IndexCommands(Index index) {
        this.index = index;
    }

    public Command spinSpindexer(DoubleSupplier volt) {
        return index.run(() -> index.setSpindexVolt(volt.getAsDouble()))
                .finallyDo(() -> index.setSpindexVolt(0))
                .withName("Rotate Spindex");
    }

    public Command spinSpindexer(double volt) {
        return spinSpindexer(() -> volt);
    }

    public Command spinIndexer(DoubleSupplier volt) {
        return index.run(() -> index.setIndexerVolt(volt.getAsDouble()))
                .finallyDo(() -> index.setIndexerVolt(0))
                .withName("Rotate Indexer");
    }

    public Command spinIndexer(double volt) {
        return spinIndexer(() -> volt);
    }

    public Command spinBoth(DoubleSupplier indexerVolt, DoubleSupplier spindexVolt) {
        return index.run(() -> {
            index.setIndexerVolt(indexerVolt.getAsDouble());
            index.setSpindexVolt(spindexVolt.getAsDouble());
        }).finallyDo(index::stop)
                .withName("Set voltage for both motors");
    }

    public Command spinBoth(double indexerVolt, double spindexVolt) {
        return spinBoth(() -> indexerVolt, () -> spindexVolt);
    }

    public Command manualController(DoubleSupplier volt) {
        return spinBoth(() -> volt.getAsDouble() * MAX_INDEXER_VOLT, () -> volt.getAsDouble() * MAX_SPINDEX_VOLT);
    }
}
