package frc.robot.subsystems.index;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.tunables.extensions.TunableCommand;
import team2679.atlantiskit.valueholders.DoubleHolder;

import static frc.robot.subsystems.index.IndexConstants.*;

public class IndexCommands {
    private Index index;

    public IndexCommands(Index index) {
        this.index = index;
        TunablesManager.add("TunableSetVoltages/IndexSetVoltage", tunableSetVoltage().fullTunable());
    }

    public Command spinBoth(DoubleSupplier indexerVolt, DoubleSupplier spindexVolt) {
        return index.run(() -> {
            index.setIndexerVolt(indexerVolt.getAsDouble());
            index.setSpindexVolt(spindexVolt.getAsDouble());
        }).finallyDo(index::stop)
                .withName("Index spin motors");
    }
    public Command stopBoth(){
        return spinBoth(0,0);
    }

    public Command spinBoth(double indexerVolt, double spindexVolt) {
        return spinBoth(() -> indexerVolt, () -> spindexVolt);
    }

    public Command manualController(DoubleSupplier speed) {
        return spinBoth(() -> speed.getAsDouble() * MAX_INDEXER_VOLT, () -> speed.getAsDouble() * MAX_SPINDEX_VOLT)
                .withName("Index manual controller");
    }

    private TunableCommand tunableSetVoltage() {
        return TunableCommand.wrap((tunablesTable) -> {
            DoubleHolder voltageIndexer = tunablesTable.addNumber("voltageIndexer", 0.0);
            DoubleHolder voltageSpindex = tunablesTable.addNumber("voltageSpindex", 0.0);
            return spinBoth(voltageIndexer::get, voltageSpindex::get)
                .withName("Tunable Flywheel set voltage");
        });
    }
}
