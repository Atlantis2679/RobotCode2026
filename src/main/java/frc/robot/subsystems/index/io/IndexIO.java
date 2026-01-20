package frc.robot.subsystems.index.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class IndexIO extends IOBase {
    public DoubleSupplier getCenterCurrent = fields.addDouble("Center Current", this::getSpindexCurrent);
    public DoubleSupplier getAddCurrent = fields.addDouble("Additional Current", this::getIndexerCurrent);

    public IndexIO(LogFieldsTable fields){
        super(fields);
    }

    //Output:
    protected abstract double getSpindexCurrent();
    protected abstract double getIndexerCurrent();

    //Input:
    public abstract void setSpindexVolt(double volt);
    public abstract void setIndexerVolt(double volt);
}
