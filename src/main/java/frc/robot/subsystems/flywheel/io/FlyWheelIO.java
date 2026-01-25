package frc.robot.subsystems.flywheel.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class FlyWheelIO extends IOBase{
    
    public final DoubleSupplier flywheelMotorAbsoluteRotations = fields.addDouble("FlyWheel rpm",
        this::getAbsoluteRotations);


    public FlyWheelIO(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    public abstract double getAbsoluteRotations();


    public abstract void setVoltage(double volt);

}
