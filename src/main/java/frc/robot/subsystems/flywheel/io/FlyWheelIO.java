package frc.robot.subsystems.flywheel.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class FlyWheelIO extends IOBase{
    
    public final DoubleSupplier getMotorsRPM = fields.addDouble("FlyWheel rpm", this::getMotorsRPM);

    public FlyWheelIO(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    public abstract void setVoltage(double volt);
        
    protected abstract double getMotorsRPM();

}
