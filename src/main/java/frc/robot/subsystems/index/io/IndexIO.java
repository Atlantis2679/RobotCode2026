package frc.robot.subsystems.index.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class IndexIO extends IOBase {
    public DoubleSupplier spindexCurrent = fields.addDouble("Spindex Current", this::getSpindexCurrent);
    public DoubleSupplier indexerCurrent = fields.addDouble("Indexer Current", this::getIndexerCurrent);

    public IndexIO(LogFieldsTable fields){
        super(fields);
    }

    protected abstract double getSpindexCurrent();
    protected abstract double getIndexerCurrent();

    public abstract void setSpindexVolt(double volt);
    public abstract void setIndexerVolt(double volt);
}
