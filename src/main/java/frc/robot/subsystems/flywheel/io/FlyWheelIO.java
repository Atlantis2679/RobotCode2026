package frc.robot.subsystems.flywheel.io;

import java.util.function.DoubleSupplier;

import team2679.atlantiskit.logfields.IOBase;
import team2679.atlantiskit.logfields.LogFieldsTable;

public abstract class FlyWheelIO extends IOBase{
    public final DoubleSupplier motorsRPM = fields.addDouble("motorsRPM", this::getMotorsRPM);
    public final DoubleSupplier motor1Current = fields.addDouble("motor1Current", this::getMotor1Current);
    public final DoubleSupplier motor2Current = fields.addDouble("motor2Current", this::getMotor2Current);

    public FlyWheelIO(LogFieldsTable fieldsTable){
        super(fieldsTable);
    }

    public abstract void setVoltage(double volt);
        
    protected abstract double getMotorsRPM();
    protected abstract double getMotor1Current();
    protected abstract double getMotor2Current();
}
