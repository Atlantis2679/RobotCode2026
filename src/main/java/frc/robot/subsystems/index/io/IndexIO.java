package frc.robot.subsystems.index.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class IndexIO extends IOBase {
    public DoubleSupplier getSpinCurrent = fields.addDouble("Spin Current", this::getSpindexCurrent);
    public DoubleSupplier getInCurrent = fields.addDouble("In Current", this::getIndexerCurrent);

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
